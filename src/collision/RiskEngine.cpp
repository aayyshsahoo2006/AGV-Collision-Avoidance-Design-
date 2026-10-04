#include "../../include/collision/RiskEngine.hpp"
#include "../../include/collision/Distance.hpp"
#include "../../include/collision/StoppingDistance.hpp"
#include "../../include/collision/TTC.hpp"
#include <sstream>
#include <limits>

namespace agv {
namespace collision {

std::string RiskAssessment::toString() const {
    std::ostringstream ss;
    ss << "RiskAssessment[State=" << controller::toString(recommended_state)
       << ", ObstacleID=" << most_critical_obstacle_id
       << ", MinDist=" << (min_distance > 1e6 ? -1.0 : min_distance) << "m"
       << ", MinTTC=" << (min_ttc > 1e6 ? -1.0 : min_ttc) << "s"
       << ", InCorridor=" << (obstacle_in_corridor ? "YES" : "NO")
       << ", d_stop=" << stopping_distance << "m"
       << ", d_slow=" << slowdown_distance << "m"
       << ", d_warn=" << warning_distance << "m]";
    return ss.str();
}

RiskEngine::RiskEngine(const config::SafetyParameters& params) : params_(params) {}

RiskAssessment RiskEngine::evaluateSingle(const sensor::SensorObservation& obs,
                                          double current_velocity,
                                          double nominal_velocity) const {
    RiskAssessment assessment;
    double eval_v = std::max(current_velocity, nominal_velocity);

    assessment.stopping_distance = StoppingDistanceCalculator::calculateStoppingDistance(eval_v, params_);
    assessment.slowdown_distance = StoppingDistanceCalculator::calculateSlowdownDistance(eval_v, params_);
    assessment.warning_distance  = StoppingDistanceCalculator::calculateWarningDistance(eval_v, params_);

    if (!obs.detected) {
        assessment.recommended_state = controller::SafetyState::NORMAL;
        return assessment;
    }

    assessment.most_critical_obstacle_id = obs.obstacle_id;
    assessment.min_distance = obs.distance;
    assessment.closing_velocity = obs.relative_velocity;
    assessment.min_ttc = TTCCalculator::calculateTTC(obs.distance, obs.relative_velocity);
    assessment.obstacle_in_corridor = DistanceCalculator::isInCorridor(obs.relative_pos, params_);

    if (!assessment.obstacle_in_corridor) {
        assessment.recommended_state = controller::SafetyState::NORMAL;
        return assessment;
    }

    bool is_critical_dist = (obs.distance <= assessment.stopping_distance);
    bool is_critical_ttc  = TTCCalculator::isCriticalTTC(assessment.min_ttc, params_.critical_ttc);

    bool is_slowdown_dist = (obs.distance <= assessment.slowdown_distance);
    bool is_warning_ttc   = TTCCalculator::isWarningTTC(assessment.min_ttc, params_.warning_ttc);

    bool is_warning_dist  = (obs.distance <= assessment.warning_distance);

    if (is_critical_dist || is_critical_ttc) {
        assessment.recommended_state = controller::SafetyState::EMERGENCY_STOP;
    } else if (is_slowdown_dist || is_warning_ttc) {
        assessment.recommended_state = controller::SafetyState::SLOWDOWN;
    } else if (is_warning_dist) {
        assessment.recommended_state = controller::SafetyState::WARNING;
    } else {
        assessment.recommended_state = controller::SafetyState::NORMAL;
    }

    return assessment;
}

static int stateSeverity(controller::SafetyState s) {
    switch (s) {
        case controller::SafetyState::EMERGENCY_STOP: return 3;
        case controller::SafetyState::SLOWDOWN:       return 2;
        case controller::SafetyState::WARNING:        return 1;
        case controller::SafetyState::NORMAL:         return 0;
    }
    return 0;
}

RiskAssessment RiskEngine::evaluateRisk(const std::vector<sensor::SensorObservation>& observations,
                                        double current_velocity,
                                        double nominal_velocity) const {
    double eval_v = std::max(current_velocity, nominal_velocity);
    RiskAssessment highest_risk;
    highest_risk.stopping_distance = StoppingDistanceCalculator::calculateStoppingDistance(eval_v, params_);
    highest_risk.slowdown_distance = StoppingDistanceCalculator::calculateSlowdownDistance(eval_v, params_);
    highest_risk.warning_distance  = StoppingDistanceCalculator::calculateWarningDistance(eval_v, params_);
    highest_risk.recommended_state = controller::SafetyState::NORMAL;

    if (observations.empty()) {
        return highest_risk;
    }

    for (const auto& obs : observations) {
        RiskAssessment single = evaluateSingle(obs, current_velocity, nominal_velocity);

        int cur_sev = stateSeverity(highest_risk.recommended_state);
        int new_sev = stateSeverity(single.recommended_state);

        if (new_sev > cur_sev) {
            highest_risk = single;
        } else if (new_sev == cur_sev && single.obstacle_in_corridor) {
            if (single.min_distance < highest_risk.min_distance) {
                highest_risk = single;
            }
        }
    }

    return highest_risk;
}

}
}
