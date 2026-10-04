#ifndef SENSOR_SENSOR_DATA_HPP
#define SENSOR_SENSOR_DATA_HPP

#include "../simulation/Vector2.hpp"
#include <string>
#include <sstream>

namespace agv {
namespace sensor {

/**
 * @brief Raw observation from the forward-facing virtual radar/sensor.
 */
struct SensorObservation {
    double timestamp{0.0};
    int obstacle_id{-1};
    double distance{1e9};
    double angle{0.0};
    double relative_velocity{0.0};
    simulation::Vector2 relative_pos{0.0, 0.0};
    bool detected{false};

    std::string toString() const {
        if (!detected) return "SensorObservation[No Target Detected]";
        std::ostringstream ss;
        ss << "SensorObservation[id=" << obstacle_id
           << ", dist=" << distance << "m"
           << ", angle=" << (angle * 180.0 / M_PI) << "deg"
           << ", v_rel=" << relative_velocity << "m/s"
           << ", local_pos=" << relative_pos.toString() << "]";
        return ss.str();
    }
};

}
}

#endif
