// world.h - Couples the flight software to the simulated spacecraft.
// Shared by the QEMU firmware, the WebAssembly demo, and the unit tests.
#pragma once
#include "../core/adcs.h"
#include "plant.h"

namespace sim {

class World {
public:
    void reset(float angleDeg, float rateDps, uint32_t seed);
    void step();  // one 20 ms control cycle

    adcs::Controller ctrl;
    Plant plant;
    uint32_t steps = 0;
    float timeS() const { return static_cast<float>(steps) / adcs::kStepHz; }
};

}  // namespace sim
