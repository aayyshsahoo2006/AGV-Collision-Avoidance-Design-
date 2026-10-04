#include "../../include/sensor/VirtualSensor.hpp"
#include "../../include/collision/Distance.hpp"
#include <cmath>
#include <algorithm>

namespace agv {
namespace sensor {

VirtualSensor::VirtualSensor(const config::SafetyParameters& params, uint32_t seed)
    : params_(params),
      noise_enabled_(params.sensor_noise_enabled),
      noise_stddev_(params.sensor_noise_stddev),
      rng_(seed),
      normal_dist_(0.0, 1.0) {}

std::vector<SensorObservation> VirtualSensor::scan(const simulation::World& world, double timestamp) {
    std::vector<SensorObservation> observations;
    const auto& agv = world.getAGV();
    const auto& agv_pose = agv.getPose();
    double agv_v = agv.getLinearVelocity();
    double half_fov_rad = (params_.sensor_fov_deg * 0.5) * (M_PI / 180.0);
    double front_bumper_x = params_.agv_length * 0.5;

    for (const auto& obs : world.getObstacles()) {
        simulation::Vector2 local_center = collision::DistanceCalculator::toLocalFrame(agv_pose, obs.getPosition());

        if (local_center.x <= 0.0) {
            continue;
        }

        double azimuth = std::atan2(local_center.y, local_center.x);

        if (std::abs(azimuth) > half_fov_rad) {
            continue;
        }

        double center_dist = local_center.length();
        double surface_dist = center_dist - front_bumper_x - obs.getRadius();
        if (surface_dist < 0.0) surface_dist = 0.0;

        if (surface_dist > params_.sensor_max_range) {
            continue;
        }

        double cos_th = std::cos(-agv_pose.theta);
        double sin_th = std::sin(-agv_pose.theta);
        const auto& global_v = obs.getVelocity();
        simulation::Vector2 local_obs_v{
            global_v.x * cos_th - global_v.y * sin_th,
            global_v.x * sin_th + global_v.y * cos_th
        };

        simulation::Vector2 rel_v_vector = local_obs_v - simulation::Vector2{agv_v, 0.0};
        
        simulation::Vector2 los_unit = local_center.normalized();
        double closing_rate = -los_unit.dot(rel_v_vector);

        double measured_dist = surface_dist;
        double measured_azimuth = azimuth;
        if (noise_enabled_ && noise_stddev_ > 0.0) {
            double noise = normal_dist_(rng_) * noise_stddev_;
            measured_dist = std::max(0.0, surface_dist + noise);
            measured_azimuth += normal_dist_(rng_) * (noise_stddev_ * 0.05);
        }

        SensorObservation item;
        item.timestamp = timestamp;
        item.obstacle_id = obs.getId();
        item.distance = measured_dist;
        item.angle = measured_azimuth;
        item.relative_velocity = closing_rate;
        item.relative_pos = local_center;
        item.detected = true;

        observations.push_back(item);
    }

    return observations;
}

}
}
