#ifndef SIMULATION_WORLD_HPP
#define SIMULATION_WORLD_HPP

#include "AGV.hpp"
#include "Obstacle.hpp"
#include "../config/SafetyParameters.hpp"
#include <vector>
#include <memory>

namespace agv {
namespace simulation {

class World {
public:
    explicit World(const config::SafetyParameters& params);

    void addObstacle(const Obstacle& obs);
    void clearObstacles();

    void step(const controller::VelocityCommand& cmd, double dt);

    double getSimTime() const { return sim_time_; }
    AGV& getAGV() { return agv_; }
    const AGV& getAGV() const { return agv_; }

    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    std::vector<Obstacle>& getObstacles() { return obstacles_; }

    bool checkPhysicalCollision() const;

    void reset();

private:
    config::SafetyParameters params_;
    AGV agv_;
    std::vector<Obstacle> obstacles_;
    double sim_time_{0.0};
};

}
}

#endif
