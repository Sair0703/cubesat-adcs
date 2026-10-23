// kernel.h - A small preemptive real-time kernel for ARM Cortex-M3.
//
// Features: fixed-priority preemptive scheduling with round-robin time
// slicing among equal priorities, 1 kHz tick, sleeping, counting semaphores
// with timeouts, and a type-safe message queue. All memory is static: no heap.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace os {

constexpr uint32_t kTickHz = 1000;
constexpr uint32_t kWaitForever = 0xFFFFFFFF;
constexpr int kMaxTasks = 8;

using TaskFn = void (*)(void* arg);

// ---- Critical sections: mask interrupts, restoring the previous state ----
inline uint32_t enter_critical() {
    uint32_t primask;
    asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory");
    return primask;
}
inline void exit_critical(uint32_t primask) {
    asm volatile("msr primask, %0\n isb" ::"r"(primask) : "memory");  // isb: take any pending switch now
}

class CriticalSection {
public:
    CriticalSection() : saved_(enter_critical()) {}
    ~CriticalSection() { exit_critical(saved_); }
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    uint32_t saved_;
};

// ---- Tasks ----
// Higher number = higher priority. Priority 0 is reserved for the idle task.
int task_create(TaskFn fn, void* arg, uint8_t priority, uint32_t* stack,
                size_t stackWords, const char* name);
[[noreturn]] void start();  // starts the scheduler; never returns

void delay(uint32_t ms);
void yield();
uint32_t ticks();
uint32_t context_switches();
const char* current_task_name();
uint32_t stack_high_water(int taskId);  // words of stack never used

// ---- Counting semaphore ----
class Semaphore {
public:
    constexpr explicit Semaphore(uint32_t initial, uint32_t max = 0xFFFF)
        : count_(initial), max_(max) {}
    bool wait(uint32_t timeoutMs = kWaitForever);  // false on timeout
    void signal();                                  // safe from tasks and ISRs
    uint32_t count() const { return count_; }

private:
    volatile uint32_t count_;
    const uint32_t max_;
};

// ---- Message queue: fixed capacity, copies items, blocks when full/empty ----
template <typename T, size_t N>
class Queue {
public:
    constexpr Queue() : items_(0, N), slots_(N, N) {}

    bool send(const T& item, uint32_t timeoutMs = kWaitForever) {
        if (!slots_.wait(timeoutMs)) return false;
        {
            CriticalSection cs;
            buf_[head_] = item;
            head_ = (head_ + 1) % N;
        }
        items_.signal();
        return true;
    }

    bool receive(T& out, uint32_t timeoutMs = kWaitForever) {
        if (!items_.wait(timeoutMs)) return false;
        {
            CriticalSection cs;
            out = buf_[tail_];
            tail_ = (tail_ + 1) % N;
        }
        slots_.signal();
        return true;
    }

private:
    Semaphore items_;  // filled slots
    Semaphore slots_;  // free slots
    T buf_[N] = {};
    size_t head_ = 0, tail_ = 0;
};

}  // namespace os
