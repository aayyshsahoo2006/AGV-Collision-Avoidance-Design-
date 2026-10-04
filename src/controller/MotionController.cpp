#include "../../include/controller/MotionController.hpp"
#include <algorithm>

namespace agv {
namespace controller {

MotionController::MotionController(const config::SafetyParameters& params)
    : params_(params), current_commanded_speed_(0.0) {}

VelocityCommand MotionController::computeCommand(SafetyState current_state,
                                                double nominal_speed,
                                                const collision::RiskAssessment& risk,
                                                double current_actual_speed,
                                                double dt) {
    VelocityCommand cmd;
    cmd.angular_velocity = 0.0;

    switch (current_state) {
        case SafetyState::NORMAL: {
            cmd.linear_velocity = std::min(nominal_speed, params_.max_velocity);
            cmd.emergency_brake = false;
            cmd.warning_active = false;
            break;
        }
        case SafetyState::WARNING: {
            cmd.linear_velocity = std::min(nominal_speed, params_.max_velocity);
            cmd.emergency_brake = false;
            cmd.warning_active = true;
            break;
        }
        case SafetyState::SLOWDOWN: {
            double span = risk.slowdown_distance - risk.stopping_distance;
            double factor = 0.5;
            if (span > 0.01) {
                double margin = risk.min_distance - risk.stopping_distance;
                factor = std::clamp(margin / span, 0.2, 0.6);
            }
            cmd.linear_velocity = nominal_speed * factor;
            cmd.emergency_brake = false;
            cmd.warning_active = true;
            break;
        }
        case SafetyState::EMERGENCY_STOP: {
            cmd.linear_velocity = 0.0;
            cmd.emergency_brake = true;
            cmd.warning_active = true;
            break;
        }
    }

    current_commanded_speed_ = cmd.linear_velocity;
    return cmd;
}

void MotionController::reset() {
    current_commanded_speed_ = 0.0;
}

}
}
