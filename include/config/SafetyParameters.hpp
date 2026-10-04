#ifndef CONFIG_SAFETY_PARAMETERS_HPP
#define CONFIG_SAFETY_PARAMETERS_HPP

namespace agv {
namespace config {

/**
 * @brief Safety and simulation configuration parameters.
 */
struct SafetyParameters {
    double agv_length{1.0};
    double agv_width{0.6};
    double max_velocity{1.5};

    double sensor_max_range{10.0};
    double sensor_fov_deg{90.0};
    double sensor_update_rate_hz{20.0};
    bool sensor_noise_enabled{false};
    double sensor_noise_stddev{0.05};

    double reaction_time{0.2};
    double deceleration{1.0};
    double safety_margin{0.3};
    double slowdown_margin{1.0};
    double warning_margin{2.0};

    double critical_ttc{1.0};
    double warning_ttc{2.5};

    double corridor_lateral_margin{0.2};
    double recovery_clear_time{1.0};
    double recovery_distance_buffer{0.5};

    double corridorHalfWidth() const {
        return (agv_width / 2.0) + corridor_lateral_margin;
    }
};

}
}

#endif
