// plant.h - Simulated spacecraft physics and sensors (the "world" the flight
// software controls). Uses floating point: it stands in for real hardware.
#pragma once
#include <stdint.h>

namespace sim {

struct PlantConfig {
    float bodyInertia = 0.05f;      // kg*m^2, a 6U CubeSat about one axis
    float wheelInertia = 5e-5f;     // kg*m^2
    float wheelMaxRpm = 6000.0f;
    float torqueLimitMnm = 4.0f;
    float disturbanceMnm = 0.02f;   // constant environmental torque
    float gyroNoiseDps = 0.02f;
    float gyroBiasDps = 0.05f;
    float sunNoiseDeg = 0.3f;
};

enum SensorFault : uint8_t { kSimFaultNone = 0, kSimFaultSunStuck = 1, kSimFaultSunBias = 2 };

struct SensorReadings {
    float gyroDps, sunDeg, wheelRpm;
};

class Plant {
public:
    void reset(float angleDeg, float rateDps, uint32_t seed);
    void step(float dt, float wheelTorqueMnm, float magnetorquerMnm = 0);
    SensorReadings read();
    void kick(float deltaRateDps) { rateDps_ += deltaRateDps; }
    void setFault(uint8_t f);

    float angleDeg() const { return angleDeg_; }
    float rateDps() const { return rateDps_; }
    float wheelRpm() const { return wheelRpm_; }
    float appliedTorqueMnm() const { return appliedMnm_; }
    uint8_t fault() const { return fault_; }

    PlantConfig cfg;

private:
    float gauss();
    float angleDeg_ = 0, rateDps_ = 0, wheelRpm_ = 0, appliedMnm_ = 0, frozenSun_ = 0;
    uint32_t rng_ = 1;
    uint8_t fault_ = kSimFaultNone;
};

float wrap180(float deg);

}  // namespace sim
