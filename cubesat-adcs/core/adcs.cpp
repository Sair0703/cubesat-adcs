// adcs.cpp - Attitude estimation, control laws, mode logic, fault detection.
#include "adcs.h"

namespace adcs {
namespace {
constexpr Fx kDt = Fx::fromFloat(1.0f / kStepHz);

// Complementary filter: trust the gyro short-term, the sun sensor long-term.
constexpr Fx kSunGain = Fx::fromFloat(0.02f);

// Control gains for a 0.05 kg*m^2 CubeSat: ~1 rad/s bandwidth, damping 0.8.
constexpr Fx kP = Fx::fromFloat(0.87f);    // mNm per degree of error
constexpr Fx kD = Fx::fromFloat(1.40f);    // mNm per deg/s of body rate
constexpr Fx kI = Fx::fromFloat(0.05f);    // mNm per degree-second
constexpr Fx kIntegralLimit = Fx::fromFloat(1.0f);   // mNm
constexpr Fx kIntegralZone = Fx::fromFloat(2.0f);    // only integrate near the target
constexpr Fx kTorqueLimit = Fx::fromFloat(4.0f);     // mNm, wheel motor limit

// Slew rate limit: clamping the error so kP*err never exceeds kD*maxRate
// makes large slews cruise at ~6 deg/s instead of saturating the wheel.
constexpr Fx kMaxSlewDps = Fx::fromFloat(6.0f);
constexpr Fx kErrorClamp = kD * kMaxSlewDps / kP;  // about 9.7 degrees

constexpr Fx kCalmRate = Fx::fromFloat(0.3f);        // deg/s: "detumbled" threshold
constexpr int32_t kCalmSteps = kStepHz;              // must stay calm for 1 s
constexpr int32_t kStuckSteps = kStepHz / 2;         // 0.5 s of identical readings
constexpr Fx kMismatchDeg = Fx::fromFloat(15.0f);
constexpr int32_t kMismatchSteps = 10;
constexpr Fx kWheelWarnRpm = Fx::fromInt(5400);
constexpr Fx kWheelOkRpm = Fx::fromInt(4800);

// Momentum dumping: when stored wheel momentum gets high, the magnetorquers
// apply a small external torque; the attitude loop holds pointing by spinning
// the wheel down against it.
constexpr Fx kDumpStartRpm = Fx::fromInt(3000);
constexpr Fx kDumpStopRpm = Fx::fromInt(300);
constexpr Fx kMagnetorquerMnm = Fx::fromFloat(0.8f);
}  // namespace

void Controller::reset(Fx initialAngleDeg) {
    tm_ = Telemetry{Mode::Detumble, kFaultNone, wrapDeg(initialAngleDeg), target_, Fx{}, Fx{}, false};
    est_ = wrapDeg(initialAngleDeg);
    lastSun_ = Fx::max();
    integral_ = Fx{};
    stuckCount_ = mismatchCount_ = calmCount_ = 0;
}

void Controller::clearFaults() {
    tm_.faults &= kFaultWheelSat;  // keep the live wheel warning
    tm_.mode = Mode::Detumble;
    stuckCount_ = mismatchCount_ = calmCount_ = 0;
    resync_ = true;  // re-anchor the estimate to the sun sensor on the next step
}

void Controller::estimate(const Sensors& s) {
    const Fx predicted = wrapDeg(est_ + s.gyroRateDps * kDt);  // propagate with the gyro
    const Fx innovation = wrapDeg(s.sunAngleDeg - predicted);  // what the sun sensor says
    monitorFaults(s, innovation);
    if (resync_) {
        est_ = wrapDeg(s.sunAngleDeg);
        resync_ = false;
    } else if (tm_.mode == Mode::Safe) {
        est_ = predicted;  // sun sensor is suspect: gyro-only dead reckoning
    } else {
        est_ = wrapDeg(predicted + kSunGain * innovation);
    }
}

void Controller::monitorFaults(const Sensors& s, Fx innovation) {
    // A real sensor always has noise; identical readings mean it's frozen.
    stuckCount_ = (s.sunAngleDeg == lastSun_) ? stuckCount_ + 1 : 0;
    lastSun_ = s.sunAngleDeg;
    if (stuckCount_ >= kStuckSteps) tm_.faults |= kFaultSunStuck;

    // A large, persistent disagreement with the gyro means the sun sensor is lying.
    if (!resync_) {
        mismatchCount_ = (fxAbs(innovation) > kMismatchDeg) ? mismatchCount_ + 1 : 0;
        if (mismatchCount_ >= kMismatchSteps) tm_.faults |= kFaultSunMismatch;
    }

    // Wheel speed warning with hysteresis.
    const Fx wheel = fxAbs(s.wheelSpeedRpm);
    if (wheel > kWheelWarnRpm) tm_.faults |= kFaultWheelSat;
    if (wheel < kWheelOkRpm) tm_.faults &= static_cast<uint8_t>(~kFaultWheelSat);

    if ((tm_.faults & (kFaultSunStuck | kFaultSunMismatch)) && tm_.mode != Mode::Safe) {
        tm_.mode = Mode::Safe;
        integral_ = Fx{};
    }
}

Fx Controller::rateDamping(Fx rate) const { return -(kD * rate); }

Fx Controller::pointing(Fx rate) {
    const Fx err = wrapDeg(target_ - est_);
    const Fx errClamped = fxClamp(err, -kErrorClamp, kErrorClamp);
    if (fxAbs(err) < kIntegralZone) {
        integral_ = fxClamp(integral_ + kI * err * kDt, -kIntegralLimit, kIntegralLimit);
    }
    // Derivative acts on the measured rate (not the error) to avoid kicks on new targets.
    return kP * errClamped + integral_ - kD * rate;
}

Fx Controller::momentumManagement(Fx wheelRpm) {
    const Fx speed = fxAbs(wheelRpm);
    if (!tm_.dumping && speed > kDumpStartRpm) tm_.dumping = true;
    if (tm_.dumping && speed < kDumpStopRpm) tm_.dumping = false;
    if (!tm_.dumping) return Fx{};
    // Push the body the way that makes the attitude loop slow the wheel down.
    return wheelRpm > Fx{} ? -kMagnetorquerMnm : kMagnetorquerMnm;
}

Commands Controller::step(const Sensors& s) {
    estimate(s);

    Fx torque{};
    switch (tm_.mode) {
        case Mode::Detumble:
            torque = rateDamping(s.gyroRateDps);
            calmCount_ = (fxAbs(s.gyroRateDps) < kCalmRate) ? calmCount_ + 1 : 0;
            if (calmCount_ >= kCalmSteps) {
                tm_.mode = Mode::Pointing;
                integral_ = Fx{};
            }
            break;
        case Mode::Pointing:
            torque = pointing(s.gyroRateDps);
            break;
        case Mode::Safe:
            torque = rateDamping(s.gyroRateDps);
            break;
    }
    torque = fxClamp(torque, -kTorqueLimit, kTorqueLimit);

    tm_.estAngleDeg = est_;
    tm_.targetDeg = target_;
    tm_.torqueCmdMnm = torque;
    tm_.magnetorquerMnm = momentumManagement(s.wheelSpeedRpm);
    return Commands{torque, tm_.magnetorquerMnm};
}

}  // namespace adcs
