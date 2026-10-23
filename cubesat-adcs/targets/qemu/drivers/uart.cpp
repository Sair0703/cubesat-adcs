// uart.cpp - Polling UART0 driver. Registers per the LM3S6965 datasheet.
#include "uart.h"

namespace {
constexpr uint32_t kUart0Base = 0x4000C000;
constexpr uint32_t kSysctlRcgc1 = 0x400FE104;  // clock gating for UART modules
constexpr uint32_t kSysctlRcgc2 = 0x400FE108;  // clock gating for GPIO ports

inline volatile uint32_t& reg(uint32_t addr) { return *reinterpret_cast<volatile uint32_t*>(addr); }

inline volatile uint32_t& UART_DR()   { return reg(kUart0Base + 0x000); }  // data
inline volatile uint32_t& UART_FR()   { return reg(kUart0Base + 0x018); }  // flags
inline volatile uint32_t& UART_IBRD() { return reg(kUart0Base + 0x024); }  // integer baud divisor
inline volatile uint32_t& UART_FBRD() { return reg(kUart0Base + 0x028); }  // fractional baud divisor
inline volatile uint32_t& UART_LCRH() { return reg(kUart0Base + 0x02C); }  // line control
inline volatile uint32_t& UART_CTL()  { return reg(kUart0Base + 0x030); }  // control

constexpr uint32_t kFrTxFull = 1u << 5;
}  // namespace

void uart_init() {
    reg(kSysctlRcgc1) |= 1u << 0;  // enable UART0 clock
    reg(kSysctlRcgc2) |= 1u << 0;  // enable GPIO port A clock (UART0 pins)
    UART_CTL() = 0;                // disable while configuring
    // 115200 baud from a 12 MHz clock: divisor = 12e6 / (16 * 115200) = 6.51
    UART_IBRD() = 6;
    UART_FBRD() = 33;              // round(0.51 * 64)
    UART_LCRH() = (0x3u << 5) | (1u << 4);       // 8 data bits, FIFOs enabled
    UART_CTL() = (1u << 0) | (1u << 8) | (1u << 9);  // UART, TX, RX enable
}

void uart_putc(char c) {
    while (UART_FR() & kFrTxFull) {}  // wait for room in the TX FIFO
    UART_DR() = static_cast<uint32_t>(c);
}

void uart_puts(const char* s) {
    while (*s) uart_putc(*s++);
}

void uart_put_uint(uint32_t value) {
    char digits[10];
    int n = 0;
    do {
        digits[n++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value);
    while (n) uart_putc(digits[--n]);
}
