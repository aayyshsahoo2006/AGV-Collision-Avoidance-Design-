#include "../../include/collision/Distance.hpp"
#include <cmath>
#include <algorithm>

namespace agv {
namespace collision {

simulation::Vector2 DistanceCalculator::toLocalFrame(const simulation::Pose& agv_pose,
                                                     const simulation::Vector2& world_point) {
    simulation::Vector2 diff = world_point - agv_pose.position;
    double cos_th = std::cos(-agv_pose.theta);
    double sin_th = std::sin(-agv_pose.theta);
    
    return {
        diff.x * cos_th - diff.y * sin_th,
        diff.x * sin_th + diff.y * cos_th
    };
}

bool DistanceCalculator::isInCorridor(const simulation::Vector2& local_point,
                                      const config::SafetyParameters& params) {
    if (local_point.x <= 0.0) {
        return false;
    }
    double half_corridor = params.corridorHalfWidth();
    return std::abs(local_point.y) <= half_corridor;
}

double DistanceCalculator::calculateClearance(double radial_distance,
                                              double agv_front_offset,
                                              double obstacle_radius) {
    double clearance = radial_distance - agv_front_offset - obstacle_radius;
    return std::max(0.0, clearance);
}

}
}
