#include "control.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void expect_command(float tilt, float rate, const ControlConfig& config, float expected)
{
    float command = 99.0f;
    check(compute_control_command(tilt, rate, command, config), "Valid input rejected");
    check(std::abs(command - expected) < 1e-6f, "Unexpected control command");
}

void expect_rejected(float tilt, float rate, const ControlConfig& config)
{
    float command = 99.0f;
    check(!compute_control_command(tilt, rate, command, config), "Unsafe input accepted");
    check(command == 0.0f, "Rejected input must clear the command");
}
}  // namespace

int main()
{
    expect_command(0.2f, 0.3f, {}, 0.0f);  // Zero gains cannot move the robot.
    const ControlConfig config{2.0f, 0.5f, 0.0f, 0.4f, 0.5f};
    expect_command(0.1f, 0.0f, config, 0.2f);
    expect_command(-0.1f, 0.0f, config, -0.2f);
    expect_command(0.0f, 0.2f, config, 0.1f);
    expect_command(0.1f, -0.2f, config, 0.1f);  // Returning upright needs less drive.
    expect_command(0.4f, 1.0f, config, 0.4f);
    expect_command(-0.4f, -1.0f, config, -0.4f);

    auto offset = config;
    offset.target_tilt_rad = 0.1f;
    expect_command(0.1f, 0.0f, offset, 0.0f);
    expect_command(0.2f, 0.0f, offset, 0.2f);
    for (float tilt : {-0.6f, -0.5f, 0.5f, 0.6f}) {
        expect_rejected(tilt, 0.0f, config);
    }

    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (float invalid : {nan, infinity, -infinity}) {
        expect_rejected(invalid, 0.0f, config);
        expect_rejected(0.0f, invalid, config);
        for (auto member : {&ControlConfig::kp, &ControlConfig::kd,
                            &ControlConfig::target_tilt_rad,
                            &ControlConfig::command_limit, &ControlConfig::max_tilt_rad}) {
            auto bad = config;
            bad.*member = invalid;
            expect_rejected(0.0f, 0.0f, bad);
        }
    }
    for (auto member : {&ControlConfig::kp, &ControlConfig::kd,
                        &ControlConfig::command_limit, &ControlConfig::max_tilt_rad}) {
        auto bad = config;
        bad.*member = -0.1f;
        expect_rejected(0.0f, 0.0f, bad);
    }
    for (float invalid : {0.0f, 1.1f}) {
        auto bad = config;
        bad.command_limit = invalid;
        expect_rejected(0.0f, 0.0f, bad);
    }
    auto bad = config;
    bad.max_tilt_rad = 0.0f;
    expect_rejected(0.0f, 0.0f, bad);
    bad = config;
    bad.target_tilt_rad = config.max_tilt_rad;
    expect_rejected(0.0f, 0.0f, bad);
    bad = config;
    bad.kd = std::numeric_limits<float>::max();
    expect_rejected(0.0f, 2.0f, bad);  // Finite inputs can still overflow.
}
