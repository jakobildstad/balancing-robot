#include "control.hpp"

#include <algorithm>
#include <cmath>

bool compute_control_command(
    float tilt_rad,
    float tilt_rate_rad_s,
    float& command,
    const ControlConfig& config)
{
    // Initialize output to zero
    command = 0.0f;

    // Validate inputs and configuration parameters
    if (!std::isfinite(tilt_rad) || !std::isfinite(tilt_rate_rad_s) ||
        !std::isfinite(config.kp) || !std::isfinite(config.kd) ||
        !std::isfinite(config.target_tilt_rad) ||
        !std::isfinite(config.command_limit) ||
        !std::isfinite(config.max_tilt_rad) ||
        config.kp < 0.0f || config.kd < 0.0f ||
        config.command_limit <= 0.0f || config.command_limit > 1.0f ||
        config.max_tilt_rad <= 0.0f ||
        std::abs(config.target_tilt_rad) >= config.max_tilt_rad ||
        std::abs(tilt_rad) >= config.max_tilt_rad) {
        return false;
    }

    // Simplify notation
    float K_p = config.kp;
    float K_d = config.kd;
    float theta = tilt_rad;
    float theta_r = config.target_tilt_rad;
    float error = theta - theta_r;
    float error_d = tilt_rate_rad_s;

    // PD regulator
    const float output = K_p*error + K_d*error_d;

    // Validate output
    if (!std::isfinite(output)) {
        return false;
    }

    // Command is referenced and this is the output
    command = std::clamp(output, -config.command_limit, config.command_limit);
    return true;
}
