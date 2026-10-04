#include "../../include/controller/SafetyStateMachine.hpp"
#include "../../include/logging/Logger.hpp"

namespace agv {
namespace controller {

SafetyStateMachine::SafetyStateMachine(const config::SafetyParameters& params)
    : params_(params),
      current_state_(SafetyState::NORMAL),
      previous_state_(SafetyState::NORMAL),
      state_changed_(false),
      time_in_current_state_(0.0),
      clear_duration_(0.0) {}

SafetyState SafetyStateMachine::update(const collision::RiskAssessment& risk, double dt, double current_time) {
    if (dt <= 0.0) dt = 0.05;

    previous_state_ = current_state_;
    state_changed_ = false;
    time_in_current_state_ += dt;

    SafetyState target_state = risk.recommended_state;

    if (current_state_ == SafetyState::EMERGENCY_STOP) {
        double recovery_threshold = risk.stopping_distance + params_.recovery_distance_buffer;
        bool is_safe_for_recovery = (!risk.obstacle_in_corridor) || 
                                    (risk.min_distance > recovery_threshold && 
                                     (risk.min_ttc > params_.critical_ttc * 1.5 || risk.closing_velocity <= 0.0));

        if (is_safe_for_recovery) {
            clear_duration_ += dt;
            if (clear_duration_ >= params_.recovery_clear_time) {
                current_state_ = target_state;
                state_changed_ = true;
                time_in_current_state_ = 0.0;
                clear_duration_ = 0.0;

                logging::Logger::getInstance().logStateTransition(
                    previous_state_, current_state_, current_time,
                    "Emergency Stop cleared - recovery condition met for configured duration");
            }
        } else {
            clear_duration_ = 0.0;
        }
    } else {
        if (target_state == SafetyState::EMERGENCY_STOP) {
            current_state_ = SafetyState::EMERGENCY_STOP;
            state_changed_ = true;
            time_in_current_state_ = 0.0;
            clear_duration_ = 0.0;

            logging::Logger::getInstance().logStateTransition(
                previous_state_, current_state_, current_time,
                "Critical collision risk detected (Distance <= d_stop or TTC <= critical_ttc)");
        } else if (target_state != current_state_) {
            current_state_ = target_state;
            state_changed_ = true;
            time_in_current_state_ = 0.0;

            logging::Logger::getInstance().logStateTransition(
                previous_state_, current_state_, current_time,
                "Risk condition change to " + std::string(toString(current_state_)));
        }
    }

    return current_state_;
}

void SafetyStateMachine::reset() {
    current_state_ = SafetyState::NORMAL;
    previous_state_ = SafetyState::NORMAL;
    state_changed_ = false;
    time_in_current_state_ = 0.0;
    clear_duration_ = 0.0;
}

}
}
