#ifndef SIMULATION_AGV_HPP
#define SIMULATION_AGV_HPP

#include "Pose.hpp"
#include "Vector2.hpp"
#include "../config/SafetyParameters.hpp"
#include "../controller/MotionController.hpp"

namespace agv {
namespace simulation {

class AGV {
public:
    explicit AGV(const config::SafetyParameters& params,
                 const Pose& initial_pose = {0.0, 0.0, 0.0});

    void update(const controller::VelocityCommand& cmd, double dt);

    const Pose& getPose() const { return pose_; }
    void setPose(const Pose& pose) { pose_ = pose; }

    double getLinearVelocity() const { return linear_velocity_; }
    double getAngularVelocity() const { return angular_velocity_; }
    void setLinearVelocity(double v) { linear_velocity_ = v; }

    double getLength() const { return params_.agv_length; }
    double getWidth() const { return params_.agv_width; }

    Vector2 getFrontBumperPosition() const;
    Vector2 getRearBumperPosition() const;

    void reset(const Pose& initial_pose = {0.0, 0.0, 0.0});

private:
    config::SafetyParameters params_;
    Pose pose_;
    double linear_velocity_{0.0};
    double angular_velocity_{0.0};
};

}
}

#endif
