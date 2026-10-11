#pragma once

#include <cstdint>

#include "esp_err.h"

namespace motors {

struct CommandMapping {
    float deadband = 0.0f;  // |u| <= deadband stops the motor; range [0, 1).
    float min_duty = 0.0f;  // Duty just outside the deadband; leave zero until measured.
    float max_duty = 1.0f;  // Duty at |u| = 1; 0 <= min_duty <= max_duty <= 1.
};

struct Config {
    std::uint32_t pwm_frequency_hz = 5'000;
    float duty_limit = 0.25f;
    CommandMapping left_mapping{};
    CommandMapping right_mapping{};
};

// Call from one task at a time. Initialization leaves both outputs disabled.
// Frequency must be nonzero; duty_limit must be finite and in (0, 1].
esp_err_t init(const Config& config = {});

// Signed duty, not measured speed: positive = forward, negative = reverse.
// Finite commands are clamped to +/- duty_limit; zero disables that bridge.
// Invalid commands or driver errors trigger a best-effort stop of both motors.
esp_err_t set_duty(float left, float right);

// Normalized commands in [-1, 1], mapped per motor before applying duty_limit.
// Defaults give f(u) = u. set_duty bypasses this mapping for direct PWM tests.
esp_err_t set_command(float left, float right);

// Disables both bridges, allowing the wheels to coast; does not actively brake.
esp_err_t stop();

}  // namespace motors
