#include "../../include/collision/StoppingDistance.hpp"
#include <cmath>
#include <algorithm>

namespace agv {
namespace collision {

double StoppingDistanceCalculator::calculateStoppingDistance(double velocity,
                                                            const config::SafetyParameters& params) {
    double v = std::max(0.0, velocity);
    double d_reaction = v * params.reaction_time;
    double d_braking = (v * v) / (2.0 * std::max(0.01, params.deceleration));
    return d_reaction + d_braking + params.safety_margin;
}

double StoppingDistanceCalculator::calculateSlowdownDistance(double velocity,
                                                            const config::SafetyParameters& params) {
    return calculateStoppingDistance(velocity, params) + params.slowdown_margin;
}

double StoppingDistanceCalculator::calculateWarningDistance(double velocity,
                                                           const config::SafetyParameters& params) {
    return calculateSlowdownDistance(velocity, params) + params.warning_margin;
}

}
}
