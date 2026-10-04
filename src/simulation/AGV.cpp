#include "../../include/simulation/AGV.hpp"
#include <algorithm>

namespace agv {
namespace simulation {

AGV::AGV(const config::SafetyParameters& params, const Pose& initial_pose)
    : params_(params), pose_(initial_pose), linear_velocity_(0.0), angular_velocity_(0.0) {}

void AGV::update(const controller::VelocityCommand& cmd, double dt) {
    if (dt <= 0.0) return;

    if (cmd.emergency_brake) {
        double decel = params_.deceleration * dt;
        if (linear_velocity_ > 0.0) {
            linear_velocity_ = std::max(0.0, linear_velocity_ - decel);
        } else if (linear_velocity_ < 0.0) {
            linear_velocity_ = std::min(0.0, linear_velocity_ + decel);
        }
        angular_velocity_ = 0.0;
    } else {
        double target_v = std::clamp(cmd.linear_velocity, -params_.max_velocity, params_.max_velocity);
        double max_dv = params_.deceleration * dt;
        
        if (linear_velocity_ < target_v) {
            linear_velocity_ = std::min(target_v, linear_velocity_ + max_dv);
        } else if (linear_velocity_ > target_v) {
            linear_velocity_ = std::max(target_v, linear_velocity_ - max_dv);
        }

        angular_velocity_ = cmd.angular_velocity;
    }

    pose_.theta += angular_velocity_ * dt;
    while (pose_.theta > M_PI) pose_.theta -= 2.0 * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2.0 * M_PI;

    Vector2 forward = pose_.forwardVector();
    pose_.position += forward * (linear_velocity_ * dt);
}

Vector2 AGV::getFrontBumperPosition() const {
    double half_len = params_.agv_length / 2.0;
    return pose_.position + (pose_.forwardVector() * half_len);
}

Vector2 AGV::getRearBumperPosition() const {
    double half_len = params_.agv_length / 2.0;
    return pose_.position - (pose_.forwardVector() * half_len);
}

void AGV::reset(const Pose& initial_pose) {
    pose_ = initial_pose;
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
}

}
}
