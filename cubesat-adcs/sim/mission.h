// mission.h - The scripted test mission, shared by the QEMU firmware and the
// host tests so both check exactly the same behavior.
//   t = 0 s   spacecraft released tumbling at 8 deg/s -> detumble, then point at 0 deg
//   t = 30 s  ground commands a 90 deg slew
//   t = 60 s  sun sensor freezes (injected fault) -> must enter SAFE mode
//   t = 70 s  fault cleared by ground -> must recover and re-point at 90 deg
//   t = 95 s  mission ends
#pragma once
#include "world.h"

namespace sim {

struct MissionMetrics {
    float detumbleS = -1;      // time to reach Pointing mode
    float slewSettleS = -1;    // time from the 90 deg command to settling within 1 deg
    float overshootDeg = 0;    // worst overshoot past 90 deg
    float faultDetectS = -1;   // time from fault injection to Safe mode
    float recoverS = -1;       // time from fault clear to pointing within 1 deg again
    float maxWheelRpm = 0;
    float finalErrDeg = 0;
};

class Mission {
public:
    static constexpr float kSlewAt = 30, kFaultAt = 60, kClearAt = 70, kEndAt = 95;
    static constexpr float kTargetDeg = 90;

    void begin(uint32_t seed = 2024);
    bool step();  // advances one control cycle; false once the mission is over

    World world;
    MissionMetrics metrics;

private:
    float inBandSince_ = -1;
};

}  // namespace sim
