#include "../../include/simulation/World.hpp"
#include "../../include/collision/Distance.hpp"
#include <algorithm>

namespace agv {
namespace simulation {

World::World(const config::SafetyParameters& params)
    : params_(params), agv_(params), sim_time_(0.0) {}

void World::addObstacle(const Obstacle& obs) {
    obstacles_.push_back(obs);
}

void World::clearObstacles() {
    obstacles_.clear();
}

void World::step(const controller::VelocityCommand& cmd, double dt) {
    if (dt <= 0.0) return;

    agv_.update(cmd, dt);

    for (auto& obs : obstacles_) {
        obs.update(dt);
    }

    sim_time_ += dt;
}

bool World::checkPhysicalCollision() const {
    const Pose& agv_pose = agv_.getPose();
    double agv_half_w = params_.agv_width / 2.0;
    double agv_half_l = params_.agv_length / 2.0;

    for (const auto& obs : obstacles_) {
        Vector2 local_pos = collision::DistanceCalculator::toLocalFrame(agv_pose, obs.getPosition());
        
        double clamped_x = std::clamp(local_pos.x, -agv_half_l, agv_half_l);
        double clamped_y = std::clamp(local_pos.y, -agv_half_w, agv_half_w);
        
        double dx = local_pos.x - clamped_x;
        double dy = local_pos.y - clamped_y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq <= (obs.getRadius() * obs.getRadius())) {
            return true;
        }
    }
    return false;
}

void World::reset() {
    agv_.reset();
    obstacles_.clear();
    sim_time_ = 0.0;
}

}
}
