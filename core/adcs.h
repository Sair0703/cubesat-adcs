// adcs.h - Single-axis Attitude Determination and Control System (ADCS)
// for a small satellite with one reaction wheel. Hardware-independent:
// it takes sensor readings in and returns a torque command, once per step.
#pragma once
#include "fixed.h"

namespace adcs {

constexpr int kStepHz = 50;  // control loop rate

enum class Mode : uint8_t {
    Detumble,  // damp out rotation using the gyro only
    Pointing,  // hold or slew to the commanded attitude
    Safe,      // a sensor fault was detected: damp rotation, ignore the sun sensor
};

enum Fault : uint8_t {
    kFaultNone = 0,
    kFaultSunStuck = 1 << 0,      // sun sensor output frozen
    kFaultSunMismatch = 1 << 1,   // sun sensor disagrees with the gyro-propagated estimate
    kFaultWheelSat = 1 << 2,      // reaction wheel near its speed limit (warning)
};

struct Sensors {
    Fx gyroRateDps;     // body rate, degrees per second
    Fx sunAngleDeg;     // absolute attitude from the sun sensor, degrees
    Fx wheelSpeedRpm;   // reaction wheel speed
};

struct Telemetry {
    Mode mode;
    uint8_t faults;
    Fx estAngleDeg;
    Fx targetDeg;
    Fx torqueCmdMnm;    // commanded reaction wheel torque on the body, mNm
    Fx magnetorquerMnm; // commanded magnetorquer torque (momentum dumping), mNm
    bool dumping;       // momentum dump in progress
};

struct Commands {
    Fx wheelTorqueMnm;
    Fx magnetorquerMnm;
};

class Controller {
public:
    void reset(Fx initialAngleDeg);
    Commands step(const Sensors& s);  // one control cycle

    void commandTarget(Fx deg) { target_ = wrapDeg(deg); }
    void clearFaults();
    const Telemetry& telemetry() const { return tm_; }

private:
    void estimate(const Sensors& s);
    void monitorFaults(const Sensors& s, Fx innovation);
    Fx rateDamping(Fx rate) const;
    Fx pointing(Fx rate);
    Fx momentumManagement(Fx wheelRpm);

    Telemetry tm_{};
    Fx est_{}, target_{}, integral_{}, lastSun_{};
    int32_t stuckCount_ = 0, mismatchCount_ = 0, calmCount_ = 0;
    bool resync_ = false;
};

}  // namespace adcs
