#include "../include/config/SafetyParameters.hpp"
#include "../include/simulation/World.hpp"
#include "../include/sensor/VirtualSensor.hpp"
#include "../include/sensor/LinuxSensorInterface.hpp"
#include "../include/collision/Distance.hpp"
#include "../include/collision/RiskEngine.hpp"
#include "../include/controller/SafetyStateMachine.hpp"
#include "../include/controller/MotionController.hpp"
#include "../include/logging/Logger.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

using namespace agv;

void renderAsciiWorld(const simulation::World& world,
                     const collision::RiskAssessment& risk,
                     controller::SafetyState state,
                     double cmd_v) {
    const int GRID_X = 25;
    const int GRID_Y = 11;
    char grid[GRID_Y][GRID_X];

    for (int y = 0; y < GRID_Y; ++y) {
        for (int x = 0; x < GRID_X; ++x) {
            grid[y][x] = ' ';
        }
    }

    int mid_y = GRID_Y / 2;
    for (int x = 0; x < GRID_X; ++x) {
        grid[mid_y - 2][x] = '.';
        grid[mid_y + 2][x] = '.';
    }

    const auto& agv = world.getAGV();
    grid[mid_y][0] = '>';
    grid[mid_y - 1][0] = '[';
    grid[mid_y + 1][0] = '[';

    for (const auto& obs : world.getObstacles()) {
        auto local = collision::DistanceCalculator::toLocalFrame(agv.getPose(), obs.getPosition());
        int gx = static_cast<int>(local.x * 2.0);
        int gy = mid_y - static_cast<int>(local.y * 2.0);

        if (gx >= 0 && gx < GRID_X && gy >= 0 && gy < GRID_Y) {
            grid[gy][gx] = 'O';
        }
    }

    std::cout << "\n+------------------------- 2D RADAR SIMULATION -------------------------+\n";
    for (int y = 0; y < GRID_Y; ++y) {
        std::cout << "| ";
        for (int x = 0; x < GRID_X; ++x) {
            std::cout << grid[y][x];
        }
        std::cout << " |\n";
    }
    std::cout << "+-----------------------------------------------------------------------+\n";
    std::cout << " Time: " << std::fixed << std::setprecision(2) << world.getSimTime() << "s"
              << " | AGV Pos: " << agv.getPose().position.toString()
              << " | Speed: " << agv.getLinearVelocity() << " m/s\n"
              << " State: [" << controller::toString(state) << "]"
              << " | Cmd_V: " << cmd_v << " m/s"
              << " | Min Dist: " << (risk.min_distance > 1e6 ? -1.0 : risk.min_distance) << " m"
              << " | TTC: " << (risk.min_ttc > 1e6 ? -1.0 : risk.min_ttc) << " s\n"
              << " Stopping Dist: " << risk.stopping_distance << " m"
              << " | Slowdown Dist: " << risk.slowdown_distance << " m\n"
              << "-------------------------------------------------------------------------\n";
}

void runScenario(int scenario_id) {
    config::SafetyParameters params;
    simulation::World world(params);
    sensor::VirtualSensor sensor(params);
    sensor::LinuxSensorInterface linux_driver;
    collision::RiskEngine risk_engine(params);
    controller::SafetyStateMachine state_machine(params);
    controller::MotionController motion_controller(params);

    linux_driver.dev_open();
    logging::Logger::getInstance().initialize("output/simulation_run.log", false);

    std::cout << "\n=======================================================\n";
    std::cout << " Starting AGV Collision Avoidance Simulation Scenario " << scenario_id << "\n";
    std::cout << "=======================================================\n";

    double duration = 10.0;
    double dt = 0.05;
    double nominal_speed = 1.2;

    switch (scenario_id) {
        case 1:
            std::cout << "[Scenario 1] Nominal Free Path (No Obstacle)\n";
            duration = 4.0;
            break;
        case 2:
            std::cout << "[Scenario 2] Approaching Stationary Obstacle in Corridor (at 6.0m)\n";
            world.addObstacle(simulation::Obstacle(1, {6.0, 0.0}, {0.0, 0.0}, 0.3));
            duration = 6.0;
            break;
        case 3:
            std::cout << "[Scenario 3] Head-on Dynamic Closing Obstacle (at 8.0m closing at 0.5m/s)\n";
            world.addObstacle(simulation::Obstacle(2, {8.0, 0.0}, {-0.5, 0.0}, 0.3, simulation::ObstacleType::DYNAMIC_LINEAR));
            duration = 6.0;
            break;
        case 4:
            std::cout << "[Scenario 4] Obstacle Outside Corridor / Lateral (at 4.0m, y=1.8m)\n";
            world.addObstacle(simulation::Obstacle(3, {4.0, 1.8}, {0.0, 0.0}, 0.3));
            duration = 4.0;
            break;
        case 5:
            std::cout << "[Scenario 5] Multiple Obstacles (Lateral + In-Corridor Threat)\n";
            world.addObstacle(simulation::Obstacle(10, {3.5, 2.0}, {0.0, 0.0}, 0.3));
            world.addObstacle(simulation::Obstacle(11, {7.0, 0.0}, {-0.3, 0.0}, 0.3));
            duration = 6.0;
            break;
        case 6:
            std::cout << "[Scenario 6] Emergency Stop followed by Obstacle Clearance and Recovery\n";
            world.addObstacle(simulation::Obstacle(20, {3.0, 0.0}, {0.0, 0.0}, 0.3));
            duration = 8.0;
            break;
        default:
            std::cout << "Invalid scenario selected.\n";
            return;
    }

    int step_count = static_cast<int>(duration / dt);
    for (int step = 0; step < step_count; ++step) {
        double current_time = world.getSimTime();

        if (scenario_id == 6 && current_time >= 4.0 && !world.getObstacles().empty()) {
            std::cout << "\n>>> [EVENT at t=" << current_time << "s] Obstacle moved away from corridor! <<<\n\n";
            world.clearObstacles();
        }

        auto raw_obs = sensor.scan(world, current_time);

        linux_driver.publishObservations(raw_obs);
        auto driver_obs = linux_driver.readObservations();

        double current_v = world.getAGV().getLinearVelocity();
        auto risk = risk_engine.evaluateRisk(driver_obs, current_v, nominal_speed);

        controller::SafetyState state = state_machine.update(risk, dt, current_time);

        auto cmd = motion_controller.computeCommand(state, nominal_speed, risk, current_v, dt);

        logging::Logger::getInstance().logCycle(
            current_time,
            world.getAGV().getPose().position.x,
            world.getAGV().getPose().position.y,
            current_v,
            state,
            risk.min_distance,
            risk.min_ttc,
            cmd.linear_velocity
        );

        world.step(cmd, dt);

        if (step % 4 == 0) {
            renderAsciiWorld(world, risk, state, cmd.linear_velocity);
        }

        if (world.checkPhysicalCollision()) {
            std::cout << "\n[CRITICAL FAILURE] PHYSICAL COLLISION OCCURRED!\n";
            break;
        }
    }

    linux_driver.dev_close();
    logging::Logger::getInstance().close();
    std::cout << "\nScenario " << scenario_id << " completed successfully.\n";
}

int main(int argc, char* argv[]) {
    std::cout << "===================================================================\n";
    std::cout << " AGV Collision-Avoidance Radar Subsystem - Software Prototype\n";
    std::cout << " Architecture: C++17 / POSIX Linux Device Abstraction / Safety FSM\n";
    std::cout << "===================================================================\n";

    int selected_scenario = 2;
    if (argc > 1) {
        try {
            selected_scenario = std::stoi(argv[1]);
            if (selected_scenario < 1 || selected_scenario > 6) {
                std::cout << "[WARN] Invalid scenario number '" << argv[1] << "'. Defaulting to Scenario 2.\n";
                selected_scenario = 2;
            }
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Invalid CLI argument format (" << e.what() << "). Defaulting to Scenario 2.\n";
            selected_scenario = 2;
        }
    } else {
        std::cout << "Available Scenarios:\n"
                  << " 1: Nominal Free Path (No Obstacle)\n"
                  << " 2: Approaching Stationary Obstacle in Corridor\n"
                  << " 3: Head-on Dynamic Closing Obstacle\n"
                  << " 4: Obstacle Outside Travel Corridor\n"
                  << " 5: Multiple Obstacles Ranking\n"
                  << " 6: Emergency Stop & Controlled Recovery\n"
                  << "Running default Scenario " << selected_scenario << "...\n";
    }

    runScenario(selected_scenario);
    return 0;
}
