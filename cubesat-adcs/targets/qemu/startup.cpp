// startup.cpp - Vector table and reset handler. Runs before main(): copies
// initialized data from flash to RAM and zeroes .bss. No C runtime is used.
#include <stdint.h>
#include "drivers/uart.h"

extern "C" {
extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();
int main();
void PendSV_Handler();
void SysTick_Handler();
void SVC_Handler();

[[noreturn]] void Reset_Handler() {
    uint32_t* src = &_sidata;
    for (uint32_t* dst = &_sdata; dst < &_edata;) *dst++ = *src++;
    for (uint32_t* dst = &_sbss; dst < &_ebss;) *dst++ = 0;
    for (auto* ctor = __init_array_start; ctor < __init_array_end; ++ctor) (*ctor)();  // C++ static constructors
    main();
    for (;;) {}
}

[[noreturn]] void HardFault_Handler() {
    uart_puts("\r\n*** HARD FAULT ***\r\n");
    for (;;) {}
}

[[noreturn]] void Default_Handler() {
    uart_puts("\r\n*** UNEXPECTED INTERRUPT ***\r\n");
    for (;;) {}
}

// Cortex-M vector table: initial stack pointer, then exception handlers.
__attribute__((section(".vectors"), used))
void (* const vector_table[16])() = {
    reinterpret_cast<void (*)()>(&_estack),  // 0: initial main stack pointer
    Reset_Handler,                           // 1: reset
    Default_Handler,                         // 2: NMI
    HardFault_Handler,                       // 3: hard fault
    Default_Handler,                         // 4: memory management fault
    Default_Handler,                         // 5: bus fault
    Default_Handler,                         // 6: usage fault
    nullptr, nullptr, nullptr, nullptr,      // 7-10: reserved
    SVC_Handler,                             // 11: supervisor call (starts first task)
    Default_Handler,                         // 12: debug monitor
    nullptr,                                 // 13: reserved
    PendSV_Handler,                          // 14: PendSV (context switch)
    SysTick_Handler,                         // 15: SysTick (1 kHz scheduler tick)
};
}
