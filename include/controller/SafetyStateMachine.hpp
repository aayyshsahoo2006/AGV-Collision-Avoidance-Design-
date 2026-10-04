#ifndef CONTROLLER_SAFETY_STATE_MACHINE_HPP
#define CONTROLLER_SAFETY_STATE_MACHINE_HPP

#include "SafetyState.hpp"
#include "../collision/RiskEngine.hpp"
#include "../config/SafetyParameters.hpp"
#include <string>

namespace agv {
namespace controller {

struct StateTransitionEvent {
    SafetyState previous_state;
    SafetyState new_state;
    double timestamp;
    std::string reason;
};

class SafetyStateMachine {
public:
    explicit SafetyStateMachine(const config::SafetyParameters& params);

    /**
     * @brief Updates state machine based on current risk assessment and elapsed time delta.
     */
    SafetyState update(const collision::RiskAssessment& risk, double dt, double current_time);

    SafetyState getCurrentState() const { return current_state_; }
    SafetyState getPreviousState() const { return previous_state_; }
    bool hasStateChanged() const { return state_changed_; }
    double getTimeInCurrentState() const { return time_in_current_state_; }
    double getClearDuration() const { return clear_duration_; }

    void reset();

private:
    config::SafetyParameters params_;
    SafetyState current_state_{SafetyState::NORMAL};
    SafetyState previous_state_{SafetyState::NORMAL};
    bool state_changed_{false};
    double time_in_current_state_{0.0};
    double clear_duration_{0.0};
};

}
}

#endif
