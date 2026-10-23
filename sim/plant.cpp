// plant.cpp - Rigid-body rotation with a reaction wheel, plus noisy sensors.
#include "plant.h"

namespace sim {
namespace {
constexpr float kPi = 3.14159265f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadSToRpm = 60.0f / (2.0f * kPi);
}  // namespace

float wrap180(float deg) {
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void Plant::reset(float angleDeg, float rateDps, uint32_t seed) {
    angleDeg_ = wrap180(angleDeg);
    rateDps_ = rateDps;
    wheelRpm_ = 0;
    appliedMnm_ = 0;
    rng_ = seed ? seed : 1;
    fault_ = kSimFaultNone;
}

void Plant::setFault(uint8_t f) {
    if (f == kSimFaultSunStuck && fault_ != kSimFaultSunStuck) frozenSun_ = read().sunDeg;
    fault_ = f;
}

float Plant::gauss() {
    // Sum of four uniforms: cheap, roughly normal, unit variance after scaling.
    float sum = 0;
    for (int i = 0; i < 4; ++i) {
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;  // xorshift32
        sum += static_cast<float>(rng_ & 0xFFFFFF) / 16777216.0f;
    }
    return (sum - 2.0f) * 1.7320508f;
}

void Plant::step(float dt, float wheelTorqueMnm, float magnetorquerMnm) {
    float tau = wheelTorqueMnm;
    if (tau > cfg.torqueLimitMnm) tau = cfg.torqueLimitMnm;
    if (tau < -cfg.torqueLimitMnm) tau = -cfg.torqueLimitMnm;

    // The wheel spins opposite to the torque it puts on the body. At its speed
    // limit it can't speed up further, so it loses authority in that direction.
    const bool saturated = (wheelRpm_ >= cfg.wheelMaxRpm && tau < 0) ||
                           (wheelRpm_ <= -cfg.wheelMaxRpm && tau > 0);
    if (saturated) tau = 0;
    appliedMnm_ = tau;

    // Magnetorquers push on Earth's magnetic field: an external torque (simplified
    // here to the commanded value), so unlike the wheel they change total momentum.
    const float bodyTorque = (tau + magnetorquerMnm + cfg.disturbanceMnm) / 1000.0f;  // N*m
    const float wheelAccel = -(tau / 1000.0f) / cfg.wheelInertia;   // rad/s^2
    wheelRpm_ += wheelAccel * dt * kRadSToRpm;

    rateDps_ += (bodyTorque / cfg.bodyInertia) * dt / kDegToRad;
    angleDeg_ = wrap180(angleDeg_ + rateDps_ * dt);
}

SensorReadings Plant::read() {
    SensorReadings r;
    r.gyroDps = rateDps_ + cfg.gyroBiasDps + cfg.gyroNoiseDps * gauss();
    r.sunDeg = wrap180(angleDeg_ + cfg.sunNoiseDeg * gauss());
    if (fault_ == kSimFaultSunStuck) r.sunDeg = frozenSun_;
    if (fault_ == kSimFaultSunBias) r.sunDeg = wrap180(r.sunDeg + 40.0f);
    r.wheelRpm = wheelRpm_;
    return r;
}

}  // namespace sim
