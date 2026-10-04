#ifndef SENSOR_VIRTUAL_SENSOR_HPP
#define SENSOR_VIRTUAL_SENSOR_HPP

#include "SensorData.hpp"
#include "../simulation/World.hpp"
#include "../config/SafetyParameters.hpp"
#include <vector>
#include <random>

namespace agv {
namespace sensor {

class VirtualSensor {
public:
    explicit VirtualSensor(const config::SafetyParameters& params, uint32_t seed = 42);

    /**
     * @brief Scans the simulation world and returns all valid detections in the sensor's FOV.
     */
    std::vector<SensorObservation> scan(const simulation::World& world, double timestamp);

    void setNoiseEnabled(bool enabled) { noise_enabled_ = enabled; }
    void setNoiseStdDev(double stddev) { noise_stddev_ = stddev; }

private:
    config::SafetyParameters params_;
    bool noise_enabled_{false};
    double noise_stddev_{0.05};
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<double> normal_dist_;
};

}
}

#endif
