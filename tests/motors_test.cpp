#include "board.hpp"
#include "motors.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

struct Output {
    bool active = false;
    std::uint32_t duty = 0;
    int stops = 0;
};

std::array<Output, 2> outputs{};
std::array<std::uint32_t, 31> gpio_levels{};
std::string failing_operation;
int failing_index = -1;
int timer_calls = 0;

void fail_next(const char* operation, int index = -1)
{
    failing_operation = operation;
    failing_index = index;
}

bool fails(const char* operation, int index = -1)
{
    if (failing_operation == operation && failing_index == index) {
        failing_operation.clear();
        return true;
    }
    return false;
}

void expect_disabled()
{
    check(!outputs[0].active && !outputs[1].active, "Both bridges must be disabled");
}

void start_motors()
{
    check(motors::init() == ESP_OK, "Initialization must succeed");
    check(motors::set_duty(0.2f, 0.2f) == ESP_OK, "Start must succeed");
    check(outputs[0].active && outputs[1].active, "Both bridges must be enabled");
}
}  // namespace

esp_err_t gpio_config(const gpio_config_t* config)
{
    check(config->mode == GPIO_MODE_OUTPUT, "Motor pins must be outputs");
    for (unsigned pin = 0; pin < gpio_levels.size(); ++pin) {
        if ((config->pin_bit_mask & (1ULL << pin)) != 0) {
            check(gpio_levels[pin] == 0, "GPIO outputs must start low");
        }
    }
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t pin, std::uint32_t level)
{
    if (fails("gpio", pin)) {
        return ESP_FAIL;
    }
    const bool left_direction = pin == board::LEFT_MOTOR_IN1_PIN ||
                                pin == board::LEFT_MOTOR_IN2_PIN;
    const bool right_direction = pin == board::RIGHT_MOTOR_IN1_PIN ||
                                 pin == board::RIGHT_MOTOR_IN2_PIN;
    if ((left_direction || right_direction) && gpio_levels[pin] != level) {
        check(!outputs[right_direction ? 1 : 0].active,
              "Direction must never change with the bridge enabled");
    }
    gpio_levels.at(pin) = level;
    return ESP_OK;
}

esp_err_t ledc_timer_config(const ledc_timer_config_t* config)
{
    ++timer_calls;
    check(config->speed_mode == LEDC_LOW_SPEED_MODE, "C6 requires low speed mode");
    check(config->freq_hz == 5000, "Default PWM frequency must be 5 kHz");
    check(config->duty_resolution == 10, "PWM resolution must be 10 bits");
    return fails("timer") ? ESP_FAIL : ESP_OK;
}

esp_err_t ledc_channel_config(const ledc_channel_config_t* config)
{
    check(config->duty == 0, "PWM must start at zero duty");
    check(config->gpio_num == (config->channel == 0
        ? board::LEFT_MOTOR_ENABLE_PIN : board::RIGHT_MOTOR_ENABLE_PIN),
        "PWM must use the enable pin");
    return fails("channel", config->channel) ? ESP_FAIL : ESP_OK;
}

esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t channel, std::uint32_t idle)
{
    check(idle == 0, "Stop must force EN low");
    ++outputs.at(channel).stops;
    if (fails("stop", channel)) {
        return ESP_FAIL;
    }
    outputs.at(channel).active = false;
    return ESP_OK;
}

esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t channel, std::uint32_t duty)
{
    if (fails("duty", channel)) {
        return ESP_FAIL;
    }
    check(duty <= 1024, "PWM duty exceeds its range");
    outputs.at(channel).duty = duty;
    return ESP_OK;
}

esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t channel)
{
    if (fails("update", channel)) {
        return ESP_FAIL;
    }
    outputs.at(channel).active = outputs.at(channel).duty != 0;
    return ESP_OK;
}

int main(int argc, char** argv)
{
    check(argc == 2, "Pass a test scenario");
    const std::string scenario = argv[1];
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    if (scenario == "invalid_config") {
        for (const float limit : {nan, infinity, -0.1f, 0.0f, 1.1f}) {
            check(motors::init({5000, limit}) == ESP_ERR_INVALID_ARG,
                  "Invalid duty limit must be rejected");
        }
        check(motors::init({0, 0.25f}) == ESP_ERR_INVALID_ARG,
              "Zero frequency must be rejected");
        check(timer_calls == 0, "Invalid configuration must not configure PWM");
        check(motors::set_duty(0.2f, 0.2f) == ESP_ERR_INVALID_STATE,
              "Commands before initialization must be rejected");
        check(motors::stop() == ESP_ERR_INVALID_STATE, "Stop before init must fail");
        check(motors::init() == ESP_OK, "Valid configuration must still work");
        expect_disabled();
    } else if (scenario == "startup") {
        check(motors::init() == ESP_OK, "Initialization must succeed");
        expect_disabled();
        check(timer_calls == 1, "Both motors must share one timer");
        check(motors::set_duty(0.2f, 0.2f) == ESP_OK, "Start must succeed");
        check(motors::init() == ESP_ERR_INVALID_STATE, "Reinitialization must fail");
        expect_disabled();
    } else if (scenario == "commands") {
        start_motors();
        check(outputs[0].duty == 205 && outputs[1].duty == 205,
              "20 percent duty must map to 205 ticks");
        check(motors::set_duty(10.0f, -10.0f) == ESP_OK, "Clamping must succeed");
        check(outputs[0].duty == 256 && outputs[1].duty == 256,
              "Duty must be clamped to 25 percent on both channels");
        check(gpio_levels[board::LEFT_MOTOR_IN1_PIN] == !board::LEFT_MOTOR_INVERTED,
              "Positive left direction is incorrect");
        check(gpio_levels[board::RIGHT_MOTOR_IN1_PIN] == board::RIGHT_MOTOR_INVERTED,
              "Negative right direction is incorrect");
        check(motors::set_duty(-0.1f, 0.2f) == ESP_OK, "Reversal must succeed");
        check(outputs[0].duty == 102 && outputs[1].duty == 205,
              "Motor duties must be independent");
        check(gpio_levels[board::LEFT_MOTOR_IN1_PIN] == board::LEFT_MOTOR_INVERTED,
              "Negative left direction is incorrect");
        check(gpio_levels[board::RIGHT_MOTOR_IN1_PIN] == !board::RIGHT_MOTOR_INVERTED,
              "Positive right direction is incorrect");
        check(motors::set_duty(0.0f, 0.2f) == ESP_OK, "Single motor stop must work");
        check(!outputs[0].active && outputs[1].active, "Zero must stop only its motor");
        check(motors::set_duty(0.000001f, -0.000001f) == ESP_OK,
              "Sub-resolution commands must succeed");
        expect_disabled();
        check(motors::set_duty(0.1f, 0.1f) == ESP_OK, "Restart after stop must work");
        check(outputs[0].active && outputs[1].active, "PWM must resume after stop");
        check(motors::stop() == ESP_OK, "Stop must succeed");
        check(motors::stop() == ESP_OK, "Repeated stop must succeed");
        expect_disabled();
    } else if (scenario == "single_motor_test") {
        check(motors::init({5000, 0.5f}) == ESP_OK, "Test configuration must succeed");
        expect_disabled();
        check(motors::set_duty(0.5f, 0.0f) == ESP_OK, "Left motor test must start");
        check(outputs[0].active && outputs[0].duty == 512 && !outputs[1].active,
              "Left test must drive only the left motor at 50 percent");
        check(motors::stop() == ESP_OK, "Left motor test must stop");
        expect_disabled();
        check(motors::set_duty(0.0f, 0.5f) == ESP_OK, "Right motor test must start");
        check(!outputs[0].active && outputs[1].active && outputs[1].duty == 512,
              "Right test must drive only the right motor at 50 percent");
        check(motors::stop() == ESP_OK, "Right motor test must stop");
        expect_disabled();
        check(motors::set_duty(0.0f, 1.0f) == ESP_OK, "Test limit must clamp requests");
        check(!outputs[0].active && outputs[1].duty == 512,
              "Test configuration must cap duty at 50 percent");
        check(motors::stop() == ESP_OK, "Clamped test must stop");
        expect_disabled();
    } else if (scenario == "full_duty") {
        check(motors::init({5000, 1.0f}) == ESP_OK, "Full duty limit must be valid");
        check(motors::set_duty(1.0f, -1.0f) == ESP_OK, "Full duty must succeed");
        check(outputs[0].duty == 1024 && outputs[1].duty == 1024,
              "Full duty must map to 1024 ticks");
    } else if (scenario == "invalid_command") {
        start_motors();
        for (float invalid : {nan, infinity, -infinity}) {
            for (bool invalid_left : {false, true}) {
                check(motors::set_duty(0.2f, 0.2f) == ESP_OK, "Restart must succeed");
                check(motors::set_duty(invalid_left ? invalid : 0.2f,
                                      invalid_left ? 0.2f : invalid) == ESP_ERR_INVALID_ARG,
                      "Non-finite command must fail");
                expect_disabled();
            }
        }
    } else if (scenario == "gpio_failure" || scenario == "duty_failure" ||
               scenario == "update_failure") {
        start_motors();
        if (scenario == "gpio_failure") {
            fail_next("gpio", board::RIGHT_MOTOR_IN2_PIN);
        } else {
            fail_next(scenario == "duty_failure" ? "duty" : "update", 1);
        }
        check(motors::set_duty(-0.2f, -0.2f) == ESP_FAIL, "Driver failure must propagate");
        expect_disabled();
    } else if (scenario == "stop_failure") {
        start_motors();
        const int right_stops = outputs[1].stops;
        fail_next("stop", 0);
        check(motors::stop() == ESP_FAIL, "Stop failure must propagate");
        check(outputs[1].stops == right_stops + 1 && !outputs[1].active,
              "Right stop must still be attempted when left stop fails");
        check(motors::stop() == ESP_OK, "Stop retry must succeed");
        expect_disabled();
    } else if (scenario == "timer_failure" || scenario == "channel_failure") {
        if (scenario == "timer_failure") {
            fail_next("timer");
        } else {
            fail_next("channel", 1);
        }
        check(motors::init() == ESP_FAIL, "Initialization failure must propagate");
        expect_disabled();
        check(motors::set_duty(0.2f, 0.2f) == ESP_ERR_INVALID_STATE,
              "Failed initialization must not accept commands");
        check(motors::init() == ESP_OK, "Retry after partial initialization must work");
        expect_disabled();
    } else {
        check(false, "Unknown scenario");
    }
}
