#pragma once

// Robot-frame measurements; positive tilt is rotation about +Y (nose down).
// The caller maps sensor axes to forward (+X), up (+Z), and axle (+Y).
struct TiltMeasurement {
    float accel_forward_mps2;
    float accel_up_mps2;
    float gyro_rate_rad_s;
};

struct TiltEstimate {
    float tilt_rad = 0.0f;
    bool initialized = false;
};

struct ComplementaryFilterConfig {
    // Controls how quickly the accelerometer corrects drift. Larger values
    // reduce sensitivity to brief linear acceleration but correct drift more
    // slowly; smaller values correct faster but admit more acceleration disturbance.
    float correction_time_constant_s = 1.0f;

    // Supply a bias calibrated while stationary; zero means no compensation.
    float gyro_bias_rad_s = 0.0f;
};

// The first accepted sample initializes tilt from gravity; hold still at startup.
// dt_s is the caller's measured elapsed time in seconds and must be positive.
// Returns false without changing estimate for invalid input/configuration or
// non-finite arithmetic. Intended for balancing near upright, not full rotations.
bool update_complementary_filter(
    TiltEstimate& estimate,
    const TiltMeasurement& measurement,
    float dt_s,
    const ComplementaryFilterConfig& config = {});
