// main.cpp - CubeSat ADCS flight software running on the custom RTOS.
//
//   control   (priority 3) runs the 50 Hz attitude control loop against the
//                          simulated spacecraft and publishes telemetry.
//   telemetry (priority 1) formats telemetry frames and sends them over UART.
//
// The emulator runs the mission 20x faster than real time (1 ms per 20 ms step).
#include "../../sim/mission.h"
#include "drivers/uart.h"
#include "kernel/kernel.h"

namespace {

struct TmFrame {
    uint32_t steps;
    uint8_t mode, faults;
    bool dumping;
    int32_t angleTenths, targetTenths, rateHundredths, wheelRpm, torqueHundredths;
};

sim::Mission mission;
os::Queue<TmFrame, 16> tmQueue;
volatile bool missionDone = false;
uint32_t controlStack[512], telemetryStack[512];

int32_t scaled(float v, float scale) { return static_cast<int32_t>(v * scale + (v >= 0 ? 0.5f : -0.5f)); }

void put_signed(int32_t v) {
    if (v < 0) { uart_putc('-'); v = -v; }
    uart_put_uint(static_cast<uint32_t>(v));
}

// Prints a fixed-point value stored as an integer with the given number of decimals.
void put_decimal(int32_t v, int decimals) {
    if (v < 0) { uart_putc('-'); v = -v; }
    int32_t div = decimals == 1 ? 10 : 100;
    uart_put_uint(static_cast<uint32_t>(v / div));
    uart_putc('.');
    if (decimals == 2 && v % div < 10) uart_putc('0');
    uart_put_uint(static_cast<uint32_t>(v % div));
}

const char* mode_name(uint8_t m) {
    switch (static_cast<adcs::Mode>(m)) {
        case adcs::Mode::Detumble: return "DETUMBLE";
        case adcs::Mode::Pointing: return "POINTING";
        case adcs::Mode::Safe:     return "SAFE    ";
    }
    return "?";
}

void control_task(void*) {
    mission.begin();
    uint8_t lastMode = 0xFF;
    while (mission.step()) {
        const auto& w = mission.world;
        const auto& tm = w.ctrl.telemetry();
        const uint8_t mode = static_cast<uint8_t>(tm.mode);
        if (w.steps % 50 == 0 || mode != lastMode) {  // once per second, and on every mode change
            tmQueue.send(TmFrame{w.steps, mode, tm.faults, tm.dumping,
                                 scaled(w.plant.angleDeg(), 10), scaled(tm.targetDeg.toFloat(), 10),
                                 scaled(w.plant.rateDps(), 100), scaled(w.plant.wheelRpm(), 1),
                                 scaled(tm.torqueCmdMnm.toFloat(), 100)});
            lastMode = mode;
        }
        os::delay(1);
    }
    missionDone = true;
    for (;;) os::delay(1000);
}

[[noreturn]] void semihosting_exit() {
    register uint32_t r0 asm("r0") = 0x18;
    register uint32_t r1 asm("r1") = 0x20026;
    asm volatile("bkpt 0xAB" ::"r"(r0), "r"(r1));
    for (;;) {}
}

void print_metric(const char* label, float value, const char* unit) {
    uart_puts(label);
    put_decimal(scaled(value, 100), 2);
    uart_puts(unit);
    uart_puts("\r\n");
}

void telemetry_task(void*) {
    uart_puts("  t(s)  mode      angle  target  rate(dps)  wheel(rpm)  torque(mNm)  faults\r\n");
    for (;;) {
        TmFrame f;
        if (tmQueue.receive(f, 50)) {
            put_decimal(static_cast<int32_t>(f.steps * 2), 2);  // steps * 0.02 s
            uart_puts("  ");
            uart_puts(mode_name(f.mode));
            uart_puts("  ");
            put_decimal(f.angleTenths, 1);
            uart_puts("  ");
            put_decimal(f.targetTenths, 1);
            uart_puts("  ");
            put_decimal(f.rateHundredths, 2);
            uart_puts("  ");
            put_signed(f.wheelRpm);
            uart_puts("  ");
            put_decimal(f.torqueHundredths, 2);
            uart_puts("  ");
            if (f.faults & adcs::kFaultSunStuck) uart_puts("SUN_STUCK ");
            if (f.faults & adcs::kFaultSunMismatch) uart_puts("SUN_MISMATCH ");
            if (f.faults & adcs::kFaultWheelSat) uart_puts("WHEEL_SAT ");
            if (f.dumping) uart_puts("DUMPING ");
            uart_puts("\r\n");
        } else if (missionDone) {
            const auto& m = mission.metrics;
            uart_puts("\r\nMISSION SUMMARY\r\n");
            print_metric("  detumble time:        ", m.detumbleS, " s");
            print_metric("  90 deg slew settled:  ", m.slewSettleS, " s");
            print_metric("  overshoot:            ", m.overshootDeg, " deg");
            print_metric("  fault detected in:    ", m.faultDetectS, " s");
            print_metric("  recovered in:         ", m.recoverS, " s");
            print_metric("  peak wheel speed:     ", m.maxWheelRpm, " rpm");
            print_metric("  final pointing error: ", m.finalErrDeg, " deg");
            uart_puts("  context switches:     ");
            uart_put_uint(os::context_switches());
            uart_puts("\r\nMISSION COMPLETE\r\n");
            semihosting_exit();
        }
    }
}

}  // namespace

int main() {
    uart_init();
    uart_puts("\r\nCubeSat ADCS flight software on custom Cortex-M3 RTOS\r\n");
    os::task_create(control_task, nullptr, 3, controlStack, 512, "control");
    os::task_create(telemetry_task, nullptr, 1, telemetryStack, 512, "telemetry");
    os::start();
}
