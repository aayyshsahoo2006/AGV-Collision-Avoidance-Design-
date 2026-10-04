#ifndef CONTROLLER_MOTION_CONTROLLER_HPP
#define CONTROLLER_MOTION_CONTROLLER_HPP

#include "SafetyState.hpp"
#include "../collision/RiskEngine.hpp"
#include "../config/SafetyParameters.hpp"

namespace agv {
namespace controller {

struct VelocityCommand {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    bool emergency_brake{false};
    bool warning_active{false};
};

class MotionController {
public:
    explicit MotionController(const config::SafetyParameters& params);

    /**
     * @brief Computes velocity command given the active safety state, target speed, and risk assessment.
     */
    VelocityCommand computeCommand(SafetyState current_state,
                                  double nominal_speed,
                                  const collision::RiskAssessment& risk,
                                  double current_actual_speed,
                                  double dt);

    void reset();

private:
    config::SafetyParameters params_;
    double current_commanded_speed_{0.0};
};

}
}

#endif
