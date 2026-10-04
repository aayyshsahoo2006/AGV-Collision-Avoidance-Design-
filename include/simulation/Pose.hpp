#ifndef SIMULATION_POSE_HPP
#define SIMULATION_POSE_HPP

#include "Vector2.hpp"
#include <cmath>
#include <string>
#include <sstream>

namespace agv {
namespace simulation {

/**
 * @brief 2D Pose representing position (x, y) and orientation (theta in radians).
 */
struct Pose {
    Vector2 position{0.0, 0.0};
    double theta{0.0};

    Pose() = default;
    Pose(double x, double y, double th) : position(x, y), theta(th) {}
    Pose(const Vector2& pos, double th) : position(pos), theta(th) {}

    Vector2 forwardVector() const {
        return {std::cos(theta), std::sin(theta)};
    }

    Vector2 lateralVector() const {
        return {-std::sin(theta), std::cos(theta)};
    }

    std::string toString() const {
        std::ostringstream ss;
        ss << "Pose[pos=" << position.toString() << ", theta=" << (theta * 180.0 / M_PI) << " deg]";
        return ss.str();
    }
};

}
}

#endif
