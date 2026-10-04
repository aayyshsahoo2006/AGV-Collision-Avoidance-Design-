#include "../include/config/SafetyParameters.hpp"
#include "../include/simulation/World.hpp"
#include "../include/sensor/VirtualSensor.hpp"
#include "../include/sensor/LinuxSensorInterface.hpp"
#include "../include/collision/Distance.hpp"
#include "../include/collision/StoppingDistance.hpp"
#include "../include/collision/TTC.hpp"
#include "../include/collision/RiskEngine.hpp"
#include "../include/controller/SafetyStateMachine.hpp"
#include "../include/controller/MotionController.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>

using namespace agv;

int tests_passed = 0;
int tests_failed = 0;

#define TEST_ASSERT(condition, msg) \
    do { \
        if (condition) { \
            std::cout << "  [PASS] " << msg << "\n"; \
            tests_passed++; \
        } else { \
            std::cout << "  [FAIL] " << msg << " (" << #condition << ")\n"; \
            tests_failed++; \
        } \
    } while (0)

void test_TC01_NoObstacle() {
    std::cout << "\n--- Running TC-01: No Obstacle Scenario ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.getAGV().setLinearVelocity(1.0);
    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);

    TEST_ASSERT(obs.empty(), "No sensor observations generated");
    TEST_ASSERT(risk.recommended_state == controller::SafetyState::NORMAL, "Risk engine recommends NORMAL");
    TEST_ASSERT(state == controller::SafetyState::NORMAL, "FSM state remains NORMAL");
}

void test_TC02_ObstacleOutsideFOV() {
    std::cout << "\n--- Running TC-02: Obstacle Outside FOV ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(1, {2.0, 3.5}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);

    TEST_ASSERT(obs.empty(), "Obstacle outside FOV is ignored by sensor");
    TEST_ASSERT(state == controller::SafetyState::NORMAL, "State remains NORMAL");
}

void test_TC03_ObstacleOutsideRange() {
    std::cout << "\n--- Running TC-03: Obstacle Outside Range ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(2, {12.0, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);

    TEST_ASSERT(obs.empty(), "Obstacle beyond 10m range is ignored");
    TEST_ASSERT(state == controller::SafetyState::NORMAL, "State remains NORMAL");
}

void test_TC04_ObstacleWarningBoundary() {
    std::cout << "\n--- Running TC-04: Obstacle Inside Warning Boundary ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(3, {3.5, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);

    TEST_ASSERT(!obs.empty(), "Obstacle detected");
    TEST_ASSERT(risk.recommended_state == controller::SafetyState::WARNING, "Risk recommended state is WARNING");
    TEST_ASSERT(state == controller::SafetyState::WARNING, "FSM transitioned to WARNING");
}

void test_TC05_ObstacleSlowdownBoundary() {
    std::cout << "\n--- Running TC-05: Obstacle Requiring Slowdown ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(4, {2.3, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);

    TEST_ASSERT(risk.recommended_state == controller::SafetyState::SLOWDOWN, "Risk recommended state is SLOWDOWN");
    TEST_ASSERT(state == controller::SafetyState::SLOWDOWN, "FSM transitioned to SLOWDOWN");
}

void test_TC06_CriticalObstacleEmergencyStop() {
    std::cout << "\n--- Running TC-06: Critical Obstacle (Emergency Stop) ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(5, {1.4, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);

    TEST_ASSERT(risk.recommended_state == controller::SafetyState::EMERGENCY_STOP, "Risk recommended state is EMERGENCY_STOP");
    TEST_ASSERT(state == controller::SafetyState::EMERGENCY_STOP, "FSM entered EMERGENCY_STOP");
}

void test_TC07_NonClosingObstacle() {
    std::cout << "\n--- Running TC-07: Non-Closing / Receding Obstacle ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);

    world.addObstacle(simulation::Obstacle(6, {6.0, 0.0}, {1.5, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);

    TEST_ASSERT(obs[0].relative_velocity < 0.0, "Closing rate is negative (receding)");
    TEST_ASSERT(std::isinf(risk.min_ttc), "TTC is infinity (no collision trajectory)");
}

void test_TC08_MultipleObstaclesRanking() {
    std::cout << "\n--- Running TC-08: Multiple Obstacles Priority Selection ---\n";
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);

    world.addObstacle(simulation::Obstacle(10, {2.0, 2.0}, {0.0, 0.0}, 0.3));
    world.addObstacle(simulation::Obstacle(11, {2.4, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);

    TEST_ASSERT(obs.size() == 2, "Both obstacles detected by sensor");
    TEST_ASSERT(risk.most_critical_obstacle_id == 11, "Corridor obstacle prioritized over lateral obstacle");
    TEST_ASSERT(risk.recommended_state == controller::SafetyState::SLOWDOWN, "Evaluated to SLOWDOWN");
}

void test_TC09_SensorNoiseStability() {
    std::cout << "\n--- Running TC-09: Sensor Noise Robustness ---\n";
    config::SafetyParameters params;
    params.sensor_noise_enabled = true;
    params.sensor_noise_stddev = 0.05;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(12, {4.2, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    int warning_count = 0;
    for (int i = 0; i < 50; ++i) {
        auto obs = sensor.scan(world, i * 0.05);
        auto risk = risk_engine.evaluateRisk(obs, 1.0);
        auto state = fsm.update(risk, 0.05, i * 0.05);
        if (state == controller::SafetyState::WARNING) warning_count++;
    }

    TEST_ASSERT(warning_count == 50, "Sensor noise did not cause state jittering (50/50 stable WARNING)");
}

void test_TC10_EmergencyRecoveryHysteresis() {
    std::cout << "\n--- Running TC-10: Emergency Recovery & Hysteresis ---\n";
    config::SafetyParameters params;
    params.recovery_clear_time = 0.5;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine fsm(params);

    world.addObstacle(simulation::Obstacle(15, {1.4, 0.0}, {0.0, 0.0}, 0.3));
    world.getAGV().setLinearVelocity(1.0);

    auto obs = sensor.scan(world, 0.0);
    auto risk = risk_engine.evaluateRisk(obs, 1.0);
    auto state = fsm.update(risk, 0.05, 0.0);
    TEST_ASSERT(state == controller::SafetyState::EMERGENCY_STOP, "Entered EMERGENCY_STOP");

    world.clearObstacles();
    obs = sensor.scan(world, 0.05);
    risk = risk_engine.evaluateRisk(obs, 0.0);
    
    for (int i = 1; i <= 4; ++i) {
        state = fsm.update(risk, 0.05, 0.05 * i);
    }
    TEST_ASSERT(state == controller::SafetyState::EMERGENCY_STOP, "Held in EMERGENCY_STOP during recovery timer (<0.5s)");

    for (int i = 5; i <= 11; ++i) {
        state = fsm.update(risk, 0.05, 0.05 * i);
    }
    TEST_ASSERT(state == controller::SafetyState::NORMAL, "Recovered to NORMAL after sustained clear duration >= 0.5s");
}

int main() {
    std::cout << "===========================================================\n";
    std::cout << " AGV COLLISION AVOIDANCE - AUTOMATED TEST SUITE (TC01-TC10)\n";
    std::cout << "===========================================================\n";

    test_TC01_NoObstacle();
    test_TC02_ObstacleOutsideFOV();
    test_TC03_ObstacleOutsideRange();
    test_TC04_ObstacleWarningBoundary();
    test_TC05_ObstacleSlowdownBoundary();
    test_TC06_CriticalObstacleEmergencyStop();
    test_TC07_NonClosingObstacle();
    test_TC08_MultipleObstaclesRanking();
    test_TC09_SensorNoiseStability();
    test_TC10_EmergencyRecoveryHysteresis();

    std::cout << "\n===========================================================\n";
    std::cout << " TEST SUMMARY: " << tests_passed << " Passed, " << tests_failed << " Failed.\n";
    std::cout << "===========================================================\n";

    return (tests_failed == 0) ? 0 : 1;
}
