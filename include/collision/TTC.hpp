#ifndef COLLISION_TTC_HPP
#define COLLISION_TTC_HPP

#include <limits>

namespace agv {
namespace collision {

class TTCCalculator {
public:
    /**
     * @brief Computes Time-To-Collision (TTC) in seconds.
     * TTC = distance / closing_velocity
     * If closing_velocity <= 0, returns infinity (no collision risk due to closing).
     */
    static double calculateTTC(double distance, double closing_velocity);

    /**
     * @brief Checks if TTC is below critical emergency threshold.
     */
    static bool isCriticalTTC(double ttc, double critical_ttc_threshold);

    /**
     * @brief Checks if TTC is below warning threshold.
     */
    static bool isWarningTTC(double ttc, double warning_ttc_threshold);
};

}
}

#endif
