#pragma once

struct ControlConfig {
    float kp = 0.0f;  // Command per radian of forward lean.
    float kd = 0.0f;  // Command per radian/second of forward rotation.
    float target_tilt_rad = 0.0f;
    float command_limit = 1.0f;
    float max_tilt_rad = 0.5f;  // Stop beyond about 29 degrees from upright.
};

// Positive tilt/rate is nose down; positive command must drive the wheels forward.
// Uses bias-corrected gyro rate. Returns false and sets command to zero on invalid
// input/configuration or excessive tilt; the caller must then disable balancing.
bool compute_control_command(
    float tilt_rad,
    float tilt_rate_rad_s,
    float& command,
    const ControlConfig& config = {});
