// test_adcs.cpp - Host-side unit and closed-loop tests for the flight software.
#include <cmath>
#include <cstdio>

#include "../core/adcs.h"
#include "../sim/mission.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

static void test_fixed_point() {
    CHECK(Fx::fromInt(3) + Fx::fromInt(4) == Fx::fromInt(7));
    CHECK(std::fabs((Fx::fromFloat(1.5f) * Fx::fromFloat(-2.25f)).toFloat() + 3.375f) < 1e-4f);
    CHECK(std::fabs((Fx::fromFloat(10.0f) / Fx::fromFloat(4.0f)).toFloat() - 2.5f) < 1e-4f);
    CHECK(Fx::max() + Fx::fromInt(1) == Fx::max());            // saturates, no wrap
    CHECK(Fx::fromInt(30000) * Fx::fromInt(30000) == Fx::max());
    CHECK(Fx::fromInt(1) / Fx{} == Fx::max());                  // divide by zero clips
    CHECK(wrapDeg(Fx::fromInt(190)) == Fx::fromInt(-170));
    CHECK(wrapDeg(Fx::fromInt(-190)) == Fx::fromInt(170));
    CHECK(wrapDeg(Fx::fromInt(720)) == Fx::fromInt(0));
}

static void test_torque_is_limited() {
    adcs::Controller c;
    c.reset(Fx{});
    // Huge rate in detumble: command must clip at the 4 mNm motor limit.
    Fx t = c.step(adcs::Sensors{Fx::fromInt(500), Fx{}, Fx{}}).wheelTorqueMnm;
    CHECK(std::fabs(t.toFloat() + 4.0f) < 1e-3f);
}

static void test_stuck_sensor_detected() {
    adcs::Controller c;
    c.reset(Fx{});
    for (int i = 0; i < 30; ++i) c.step(adcs::Sensors{Fx{}, Fx::fromFloat(12.5f), Fx{}});
    CHECK(c.telemetry().faults & adcs::kFaultSunStuck);
    CHECK(c.telemetry().mode == adcs::Mode::Safe);
}

static void test_biased_sensor_detected() {
    sim::World w;
    w.reset(0, 0, 7);
    for (int i = 0; i < 50 * 20; ++i) w.step();  // settle into pointing
    w.plant.setFault(sim::kSimFaultSunBias);    // sun sensor suddenly reads 40 deg off
    int stepsToSafe = -1;
    for (int i = 0; i < 100; ++i) {
        w.step();
        if (w.ctrl.telemetry().mode == adcs::Mode::Safe) { stepsToSafe = i; break; }
    }
    CHECK(stepsToSafe >= 0 && stepsToSafe < 20);
    CHECK(w.ctrl.telemetry().faults & adcs::kFaultSunMismatch);
}

static void test_momentum_dumping_recovers_saturated_wheel() {
    sim::World w;
    w.reset(0, 0, 11);
    for (int i = 0; i < 50 * 15; ++i) w.step();
    for (int k = 0; k < 4; ++k) { w.plant.kick(12); for (int i = 0; i < 50 * 2; ++i) w.step(); }
    CHECK(w.ctrl.telemetry().dumping);
    for (int i = 0; i < 50 * 60; ++i) w.step();  // one minute of dumping
    std::printf("  momentum dump: wheel %.0f rpm after 1 min of dumping, pointing error %.2f deg\n",
                w.plant.wheelRpm(), sim::wrap180(w.plant.angleDeg()));
    CHECK(std::fabs(w.plant.wheelRpm()) < 1000);
    CHECK(std::fabs(sim::wrap180(w.plant.angleDeg())) < 2.0f);
    CHECK(w.ctrl.telemetry().mode == adcs::Mode::Pointing);
}

static void test_full_mission() {
    sim::Mission m;
    m.begin();
    while (m.step()) {}
    const auto& r = m.metrics;
    std::printf("  mission: detumble %.1f s, slew settle %.1f s, overshoot %.2f deg, "
                "fault detect %.2f s, recover %.1f s, max wheel %.0f rpm, final error %.2f deg\n",
                r.detumbleS, r.slewSettleS, r.overshootDeg, r.faultDetectS, r.recoverS, r.maxWheelRpm, r.finalErrDeg);
    CHECK(r.detumbleS > 0 && r.detumbleS < 20);
    CHECK(r.slewSettleS > 0 && r.slewSettleS < 25);
    CHECK(r.overshootDeg < 2.0f);
    CHECK(r.faultDetectS > 0 && r.faultDetectS < 1.0f);
    CHECK(r.recoverS > 0 && r.recoverS < 20);
    CHECK(std::fabs(r.finalErrDeg) < 1.0f);
    CHECK(r.maxWheelRpm < 6000);
}

int main() {
    test_fixed_point();
    test_torque_is_limited();
    test_stuck_sensor_detected();
    test_biased_sensor_detected();
    test_momentum_dumping_recovers_saturated_wheel();
    test_full_mission();
    if (failures == 0) std::printf("All ADCS tests passed.\n");
    return failures ? 1 : 0;
}
