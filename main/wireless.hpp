#pragma once

#include "wireless_protocol.hpp"
#include "control.hpp"
#include "esp_err.h"

namespace wireless {

struct Telemetry {
    float tilt_rad = 0.0f;
    float rate_rad_s = 0.0f;
    float command = 0.0f;
    bool sample_valid = false;
    bool motors_ready = false;
    const char* mode = "idle";
    const char* status = "Ready";
    ControlConfig control{};
};

// Start a password-protected AP and serve the embedded page. Call once at boot.
esp_err_t init();

// Nonblocking handoff: only main.cpp executes commands and changes gains.
bool read_request(Request& request);
bool connected();
void publish(const Telemetry& telemetry);

}  // namespace wireless
