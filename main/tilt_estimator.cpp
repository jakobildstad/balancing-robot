#include "tilt_estimator.hpp"

#include <cmath>

bool update_complementary_filter(
    TiltEstimate& estimate,
    const TiltMeasurement& measurement,
    float dt_s,
    const ComplementaryFilterConfig& config)
{
    if (!std::isfinite(dt_s) || dt_s <= 0.0f ||
        !std::isfinite(config.correction_time_constant_s) ||
        config.correction_time_constant_s <= 0.0f ||
        !std::isfinite(config.gyro_bias_rad_s) ||
        !std::isfinite(measurement.accel_forward_mps2) ||
        !std::isfinite(measurement.accel_up_mps2) ||
        !std::isfinite(measurement.gyro_rate_rad_s)) {
        return false;
    }

    // A zero gravity projection gives no tilt information (e.g. free fall).
    if (measurement.accel_forward_mps2 == 0.0f &&
        measurement.accel_up_mps2 == 0.0f) {
        return false;
    }

    const float correction_interval_s = config.correction_time_constant_s + dt_s;
    if (!std::isfinite(correction_interval_s)) {
        return false;
    }

    // At rest: forward = -g*sin(tilt), up = g*cos(tilt), matching +Y gyro.
    const float accelerometer_tilt = std::atan2(
        -measurement.accel_forward_mps2, measurement.accel_up_mps2);

    if (!estimate.initialized) {
        estimate.tilt_rad = accelerometer_tilt;
        estimate.initialized = true;
        return true;
    }

    // The gyro predicts angle changes; the accelerometer slowly corrects drift.
    // This does not delay the gyro's immediate response to rotation.
    const float predicted_tilt = estimate.tilt_rad +
        (measurement.gyro_rate_rad_s - config.gyro_bias_rad_s) * dt_s;
    // With dt = 0.005 s and a 1 s time constant, alpha is approximately 0.995.
    const float alpha = config.correction_time_constant_s / correction_interval_s;
    // Sustained linear acceleration can still bias the estimate.
    const float fused_tilt = alpha * predicted_tilt +
        (1.0f - alpha) * accelerometer_tilt;
    if (!std::isfinite(fused_tilt)) {
        return false;
    }

    estimate.tilt_rad = fused_tilt;
    return true;
}
