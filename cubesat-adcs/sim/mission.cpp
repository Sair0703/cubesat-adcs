#include "mission.h"

namespace sim {
namespace {
float absf(float v) { return v < 0 ? -v : v; }
}

void Mission::begin(uint32_t seed) {
    world.reset(30.0f, 8.0f, seed);
    metrics = MissionMetrics{};
    inBandSince_ = -1;
}

bool Mission::step() {
    const float t = world.timeS();
    if (t >= kEndAt) return false;

    if (world.steps == static_cast<uint32_t>(kSlewAt * adcs::kStepHz))
        world.ctrl.commandTarget(Fx::fromFloat(kTargetDeg));
    if (world.steps == static_cast<uint32_t>(kFaultAt * adcs::kStepHz))
        world.plant.setFault(kSimFaultSunStuck);
    if (world.steps == static_cast<uint32_t>(kClearAt * adcs::kStepHz)) {
        world.plant.setFault(kSimFaultNone);
        world.ctrl.clearFaults();
    }

    world.step();

    const auto& tm = world.ctrl.telemetry();
    const float now = world.timeS();
    const float err = wrap180(world.plant.angleDeg() - kTargetDeg);
    const float wheel = absf(world.plant.wheelRpm());
    if (wheel > metrics.maxWheelRpm) metrics.maxWheelRpm = wheel;

    if (metrics.detumbleS < 0 && tm.mode == adcs::Mode::Pointing) metrics.detumbleS = now;

    if (now > kSlewAt && now < kFaultAt) {
        if (err > metrics.overshootDeg) metrics.overshootDeg = err;
        if (absf(err) < 1.0f) {
            if (inBandSince_ < 0) inBandSince_ = now;
            if (metrics.slewSettleS < 0 && now - inBandSince_ >= 2.0f) metrics.slewSettleS = inBandSince_ - kSlewAt;
        } else {
            inBandSince_ = -1;
        }
    }
    if (now > kFaultAt && metrics.faultDetectS < 0 && tm.mode == adcs::Mode::Safe)
        metrics.faultDetectS = now - kFaultAt;
    if (now > kClearAt && metrics.recoverS < 0 && tm.mode == adcs::Mode::Pointing && absf(err) < 1.0f)
        metrics.recoverS = now - kClearAt;
    metrics.finalErrDeg = err;
    return true;
}

}  // namespace sim
