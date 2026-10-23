#include "world.h"

namespace sim {

void World::reset(float angleDeg, float rateDps, uint32_t seed) {
    plant.reset(angleDeg, rateDps, seed);
    ctrl.reset(Fx::fromFloat(plant.read().sunDeg));
    steps = 0;
}

void World::step() {
    const SensorReadings r = plant.read();
    const adcs::Sensors s{Fx::fromFloat(r.gyroDps), Fx::fromFloat(r.sunDeg), Fx::fromFloat(r.wheelRpm)};
    const adcs::Commands cmd = ctrl.step(s);
    plant.step(1.0f / adcs::kStepHz, cmd.wheelTorqueMnm.toFloat(), cmd.magnetorquerMnm.toFloat());
    ++steps;
}

}  // namespace sim
