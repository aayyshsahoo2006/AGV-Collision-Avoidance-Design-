#ifndef COLLISION_RISK_ENGINE_HPP
#define COLLISION_RISK_ENGINE_HPP

#include "../sensor/SensorData.hpp"
#include "../config/SafetyParameters.hpp"
#include "../controller/SafetyState.hpp"
#include <vector>

namespace agv {
namespace collision {

struct RiskAssessment {
    controller::SafetyState recommended_state{controller::SafetyState::NORMAL};
    int most_critical_obstacle_id{-1};
    double min_distance{1e9};
    double min_ttc{1e9};
    double stopping_distance{0.0};
    double slowdown_distance{0.0};
    double warning_distance{0.0};
    bool obstacle_in_corridor{false};
    double closing_velocity{0.0};

    std::string toString() const;
};

class RiskEngine {
public:
    explicit RiskEngine(const config::SafetyParameters& params);

    /**
     * @brief Evaluates risk for a list of sensor observations and selects the most critical one.
     * Uses nominal_velocity to prevent threshold hunting/oscillation when AGV slows down.
     */
    RiskAssessment evaluateRisk(const std::vector<sensor::SensorObservation>& observations,
                                double current_velocity,
                                double nominal_velocity = 1.2) const;

    /**
     * @brief Evaluates risk for a single sensor observation.
     */
    RiskAssessment evaluateSingle(const sensor::SensorObservation& obs,
                                  double current_velocity,
                                  double nominal_velocity = 1.2) const;

    const config::SafetyParameters& getParameters() const { return params_; }
    void setParameters(const config::SafetyParameters& params) { params_ = params; }

private:
    config::SafetyParameters params_;
};

}
}

#endif
