#ifndef CONTROLLER_SAFETY_STATE_HPP
#define CONTROLLER_SAFETY_STATE_HPP

#include <string>

namespace agv {
namespace controller {

/**
 * @brief Safety State Machine operational states.
 */
enum class SafetyState {
    NORMAL,
    WARNING,
    SLOWDOWN,
    EMERGENCY_STOP
};

inline const char* toString(SafetyState state) {
    switch (state) {
        case SafetyState::NORMAL: return "NORMAL";
        case SafetyState::WARNING: return "WARNING";
        case SafetyState::SLOWDOWN: return "SLOWDOWN";
        case SafetyState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

}
}

#endif
