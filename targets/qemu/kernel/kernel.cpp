// kernel.cpp - Scheduler, context switch, tick handling, and semaphores.
//
// Context switching uses the standard Cortex-M design:
//   * Tasks run in thread mode on their own stack (PSP); the kernel and
//     interrupts use the main stack (MSP).
//   * On exception entry the hardware pushes r0-r3, r12, lr, pc, xPSR.
//     PendSV_Handler pushes the rest (r4-r11), saves the stack pointer in
//     the task's TCB, picks the next task, and restores that task's registers.
//   * PendSV and SysTick run at the lowest priority, so a switch never
//     interrupts another ISR, and real interrupts always preempt the kernel.
#include "kernel.h"

namespace os {
namespace detail {

enum class State : uint8_t { Unused, Ready, Sleeping, Blocked };

struct Tcb {
    uint32_t* sp;  // MUST stay the first member: the assembly reads/writes it at offset 0
    State state;
    uint8_t priority;
    bool hasTimeout;
    bool timedOut;
    uint32_t wakeTick;
    Semaphore* waitingOn;
    uint32_t* stackBase;
    size_t stackWords;
    const char* name;
};

Tcb tasks[kMaxTasks];
int taskCount = 0;

}  // namespace detail
}  // namespace os

using os::detail::State;
using os::detail::Tcb;
using os::detail::tasks;
using os::detail::taskCount;

// Shared with the assembly handlers, so these have C linkage.
extern "C" {
Tcb* volatile os_current_tcb = nullptr;
volatile uint8_t os_kernel_running = 0;
void os_schedule();
}

namespace {
volatile uint32_t tickCount = 0;
volatile uint32_t switchCount = 0;
constexpr uint32_t kStackFill = 0xDEADBEEF;
constexpr uint32_t kCpuHz = 12000000;  // LM3S6965 default clock in QEMU
uint32_t idleStack[64];

inline volatile uint32_t& reg(uint32_t addr) { return *reinterpret_cast<volatile uint32_t*>(addr); }
inline volatile uint32_t& SCB_ICSR()   { return reg(0xE000ED04); }
inline volatile uint32_t& SCB_SHPR3()  { return reg(0xE000ED20); }
inline volatile uint32_t& SYST_CSR()   { return reg(0xE000E010); }
inline volatile uint32_t& SYST_RVR()   { return reg(0xE000E014); }
inline volatile uint32_t& SYST_CVR()   { return reg(0xE000E018); }

inline void request_context_switch() {
    SCB_ICSR() = 1u << 28;  // set PendSV pending
    asm volatile("dsb\n isb" ::: "memory");
}

void task_exit() {
    // A task function returned: retire it and never run it again.
    {
        os::CriticalSection cs;
        os_current_tcb->state = State::Unused;
        request_context_switch();
    }
    for (;;) {}
}

void idle_task(void*) {
    for (;;) asm volatile("wfi");  // sleep until the next interrupt
}
}  // namespace

// Called from PendSV with interrupts masked: pick the highest-priority ready
// task, rotating among equal priorities (starting after the current task).
extern "C" void os_schedule() {
    const int cur = static_cast<int>(os_current_tcb - tasks);
    int best = -1;
    for (int k = 1; k <= taskCount; ++k) {
        const int i = (cur + k) % taskCount;
        if (tasks[i].state != State::Ready) continue;
        if (best < 0 || tasks[i].priority > tasks[best].priority) best = i;
    }
    if (best >= 0 && best != cur) {
        os_current_tcb = &tasks[best];
        ++switchCount;
    }
}

// ---- Exception handlers (naked: no compiler-generated prologue) ----
extern "C" __attribute__((naked)) void PendSV_Handler() {
    asm volatile(
        "mrs   r0, psp              \n"  // current task's stack pointer
        "stmdb r0!, {r4-r11}        \n"  // save the registers hardware didn't
        "ldr   r1, =os_current_tcb  \n"
        "ldr   r2, [r1]             \n"
        "str   r0, [r2]             \n"  // tcb->sp = updated stack pointer
        "cpsid i                    \n"
        "push  {r3, lr}             \n"  // keep 8-byte stack alignment for the call
        "bl    os_schedule          \n"
        "pop   {r3, lr}             \n"
        "cpsie i                    \n"
        "ldr   r1, =os_current_tcb  \n"
        "ldr   r2, [r1]             \n"
        "ldr   r0, [r2]             \n"  // next task's saved stack pointer
        "ldmia r0!, {r4-r11}        \n"
        "msr   psp, r0              \n"
        "bx    lr                   \n"  // hardware restores the rest
        ".ltorg                     \n");
}

extern "C" __attribute__((naked)) void SVC_Handler() {
    // Launches the first task: load its prepared stack and "return" into it
    // in thread mode on the process stack (EXC_RETURN = 0xFFFFFFFD).
    asm volatile(
        "movs  r0, #1               \n"
        "ldr   r1, =os_kernel_running \n"
        "strb  r0, [r1]             \n"
        "ldr   r1, =os_current_tcb  \n"
        "ldr   r2, [r1]             \n"
        "ldr   r0, [r2]             \n"
        "ldmia r0!, {r4-r11}        \n"
        "msr   psp, r0              \n"
        "ldr   lr, =0xFFFFFFFD      \n"
        "bx    lr                   \n"
        ".ltorg                     \n");
}

extern "C" void SysTick_Handler() {
    ++tickCount;
    if (!os_kernel_running) return;
    const uint32_t now = tickCount;
    for (int i = 0; i < taskCount; ++i) {
        Tcb& t = tasks[i];
        const bool due = static_cast<int32_t>(now - t.wakeTick) >= 0;  // wrap-safe
        if (t.state == State::Sleeping && due) {
            t.state = State::Ready;
        } else if (t.state == State::Blocked && t.hasTimeout && due) {
            t.timedOut = true;
            t.waitingOn = nullptr;
            t.state = State::Ready;
        }
    }
    request_context_switch();  // time slice: let equal-priority tasks rotate
}

namespace os {

int task_create(TaskFn fn, void* arg, uint8_t priority, uint32_t* stack,
                size_t stackWords, const char* name) {
    CriticalSection cs;
    if (taskCount >= kMaxTasks) return -1;

    for (size_t i = 0; i < stackWords; ++i) stack[i] = kStackFill;  // for high-water checks

    // Build the frame PendSV/SVC will "restore", as if the task had been interrupted.
    uint32_t* sp = reinterpret_cast<uint32_t*>(
        reinterpret_cast<uintptr_t>(stack + stackWords) & ~uintptr_t{7});  // AAPCS: 8-byte aligned
    *--sp = 0x01000000;                                              // xPSR: Thumb bit set
    *--sp = reinterpret_cast<uint32_t>(fn) & ~1u;                    // PC: task entry
    *--sp = reinterpret_cast<uint32_t>(&task_exit);                  // LR: if the task returns
    *--sp = 0;                                                       // R12
    *--sp = 0;                                                       // R3
    *--sp = 0;                                                       // R2
    *--sp = 0;                                                       // R1
    *--sp = reinterpret_cast<uint32_t>(arg);                         // R0: task argument
    for (int i = 0; i < 8; ++i) *--sp = 0;                           // R11..R4

    Tcb& t = tasks[taskCount];
    t = Tcb{sp, State::Ready, priority, false, false, 0, nullptr, stack, stackWords, name};
    return taskCount++;
}

void start() {
    task_create(idle_task, nullptr, 0, idleStack, sizeof(idleStack) / sizeof(idleStack[0]), "idle");

    SCB_SHPR3() |= (0xFFu << 16) | (0xFFu << 24);  // PendSV and SysTick at lowest priority
    SYST_RVR() = kCpuHz / kTickHz - 1;
    SYST_CVR() = 0;
    SYST_CSR() = 0x7;  // enable, interrupt on wrap, use the CPU clock

    int first = 0;
    for (int i = 1; i < taskCount; ++i)
        if (tasks[i].priority > tasks[first].priority) first = i;
    os_current_tcb = &tasks[first];

    asm volatile("svc 0");  // never returns
    for (;;) {}
}

void delay(uint32_t ms) {
    if (ms == 0) { yield(); return; }
    CriticalSection cs;
    os_current_tcb->wakeTick = tickCount + ms;
    os_current_tcb->state = State::Sleeping;
    request_context_switch();  // takes effect as soon as interrupts are unmasked
}

void yield() { request_context_switch(); }
uint32_t ticks() { return tickCount; }
uint32_t context_switches() { return switchCount; }
const char* current_task_name() { return os_current_tcb->name; }

uint32_t stack_high_water(int taskId) {
    if (taskId < 0 || taskId >= taskCount) return 0;
    const Tcb& t = tasks[taskId];
    uint32_t unused = 0;
    while (unused < t.stackWords && t.stackBase[unused] == kStackFill) ++unused;
    return unused;
}

bool Semaphore::wait(uint32_t timeoutMs) {
    Tcb* self;
    {
        CriticalSection cs;
        if (count_ > 0) { --count_; return true; }
        if (timeoutMs == 0) return false;

        self = os_current_tcb;
        self->state = State::Blocked;
        self->waitingOn = this;
        self->timedOut = false;
        self->hasTimeout = timeoutMs != kWaitForever;
        self->wakeTick = tickCount + timeoutMs;
        request_context_switch();
    }  // interrupts unmasked here: PendSV switches away until signal() or timeout
    return !self->timedOut;
}

void Semaphore::signal() {
    CriticalSection cs;  // also safe to call from an interrupt handler
    Tcb* best = nullptr;
    for (int i = 0; i < taskCount; ++i) {
        Tcb& t = tasks[i];
        if (t.state == State::Blocked && t.waitingOn == this &&
            (!best || t.priority > best->priority))
            best = &t;
    }
    if (best) {
        // Hand the token straight to the highest-priority waiter.
        best->waitingOn = nullptr;
        best->state = State::Ready;
        if (best->priority > os_current_tcb->priority) request_context_switch();
    } else if (count_ < max_) {
        ++count_;
    }
}

}  // namespace os
