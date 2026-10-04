#ifndef COLLISION_STOPPING_DISTANCE_HPP
#define COLLISION_STOPPING_DISTANCE_HPP

#include "../config/SafetyParameters.hpp"

namespace agv {
namespace collision {

class StoppingDistanceCalculator {
public:
    /**
     * @brief Computes required emergency stopping distance:
     * d_stop = v * t_r + (v^2 / (2 * a)) + d_s
     */
    static double calculateStoppingDistance(double velocity,
                                            const config::SafetyParameters& params);

    /**
     * @brief Computes distance threshold where slowdown should begin:
     * d_slowdown = d_stop + slowdown_margin
     */
    static double calculateSlowdownDistance(double velocity,
                                            const config::SafetyParameters& params);

    /**
     * @brief Computes distance threshold where warning should be issued:
     * d_warning = d_slowdown + warning_margin
     */
    static double calculateWarningDistance(double velocity,
                                           const config::SafetyParameters& params);
};

}
}

#endif
