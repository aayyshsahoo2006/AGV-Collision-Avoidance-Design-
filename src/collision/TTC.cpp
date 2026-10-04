#include "../../include/collision/TTC.hpp"

namespace agv {
namespace collision {

double TTCCalculator::calculateTTC(double distance, double closing_velocity) {
    if (distance <= 0.0) {
        return 0.0;
    }
    if (closing_velocity <= 1e-4) {
        return std::numeric_limits<double>::infinity();
    }
    return distance / closing_velocity;
}

bool TTCCalculator::isCriticalTTC(double ttc, double critical_ttc_threshold) {
    return ttc >= 0.0 && ttc <= critical_ttc_threshold;
}

bool TTCCalculator::isWarningTTC(double ttc, double warning_ttc_threshold) {
    return ttc >= 0.0 && ttc <= warning_ttc_threshold;
}

}
}
