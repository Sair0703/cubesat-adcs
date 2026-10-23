// wasm_api.cpp - Exposes the same flight software and simulation to
// JavaScript as a WebAssembly module, so the live demo runs the real code.
#include "../../sim/world.h"

#define EXPORT extern "C" __attribute__((visibility("default")))

extern "C" void* memset(void* dst, int value, unsigned long n) {
    volatile unsigned char* d = static_cast<volatile unsigned char*>(dst);
    while (n--) *d++ = static_cast<unsigned char>(value);
    return dst;
}
extern "C" void* memcpy(void* dst, const void* src, unsigned long n) {
    volatile unsigned char* d = static_cast<volatile unsigned char*>(dst);
    const volatile unsigned char* s = static_cast<const volatile unsigned char*>(src);
    while (n--) *d++ = *s++;
    return dst;
}

namespace { sim::World world; }

EXPORT void sim_reset(float angleDeg, float rateDps, unsigned seed) { world.reset(angleDeg, rateDps, seed); }
EXPORT void sim_step() { world.step(); }
EXPORT void sim_kick(float dps) { world.plant.kick(dps); }
EXPORT void sim_set_fault(int f) { world.plant.setFault(static_cast<uint8_t>(f)); }
EXPORT void adcs_command_target(float deg) { world.ctrl.commandTarget(Fx::fromFloat(deg)); }
EXPORT void adcs_clear_faults() { world.ctrl.clearFaults(); }

EXPORT float sim_time() { return world.timeS(); }
EXPORT float sim_angle() { return world.plant.angleDeg(); }
EXPORT float sim_rate() { return world.plant.rateDps(); }
EXPORT float sim_wheel_rpm() { return world.plant.wheelRpm(); }
EXPORT int sim_fault() { return world.plant.fault(); }
EXPORT float adcs_est_angle() { return world.ctrl.telemetry().estAngleDeg.toFloat(); }
EXPORT float adcs_target() { return world.ctrl.telemetry().targetDeg.toFloat(); }
EXPORT float adcs_torque() { return world.ctrl.telemetry().torqueCmdMnm.toFloat(); }
EXPORT int adcs_mode() { return static_cast<int>(world.ctrl.telemetry().mode); }
EXPORT int adcs_faults() { return world.ctrl.telemetry().faults; }
EXPORT int adcs_dumping() { return world.ctrl.telemetry().dumping ? 1 : 0; }
EXPORT float adcs_magnetorquer() { return world.ctrl.telemetry().magnetorquerMnm.toFloat(); }
