#pragma once

#include <cstdint>

#include "esp_err.h"

namespace motors {

struct Config {
    std::uint32_t pwm_frequency_hz = 5'000;
    float duty_limit = 0.25f;
};

// Call from one task at a time. Initialization leaves both outputs disabled.
// Frequency must be nonzero; duty_limit must be finite and in (0, 1].
esp_err_t init(const Config& config = {});

// Signed duty, not measured speed: positive = forward, negative = reverse.
// Finite commands are clamped to +/- duty_limit; zero disables that bridge.
// Invalid commands or driver errors trigger a best-effort stop of both motors.
esp_err_t set_duty(float left, float right);

// Disables both bridges, allowing the wheels to coast; does not actively brake.
esp_err_t stop();

}  // namespace motors
