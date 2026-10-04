#ifndef COLLISION_DISTANCE_HPP
#define COLLISION_DISTANCE_HPP

#include "../simulation/Pose.hpp"
#include "../simulation/Vector2.hpp"
#include "../config/SafetyParameters.hpp"

namespace agv {
namespace collision {

class DistanceCalculator {
public:
    /**
     * @brief Transforms a global 2D point into the AGV's local coordinate frame.
     * Local +X points directly forward from AGV front, +Y points to the left.
     */
    static simulation::Vector2 toLocalFrame(const simulation::Pose& agv_pose,
                                            const simulation::Vector2& world_point);

    /**
     * @brief Determines if a local-frame point falls inside the AGV's forward travel corridor.
     */
    static bool isInCorridor(const simulation::Vector2& local_point,
                             const config::SafetyParameters& params);

    /**
     * @brief Computes clearance from AGV front bumper to obstacle center minus obstacle radius.
     */
    static double calculateClearance(double radial_distance,
                                    double agv_front_offset,
                                    double obstacle_radius);
};

}
}

#endif
