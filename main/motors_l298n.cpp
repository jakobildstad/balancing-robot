#include "motors.hpp"
#include "board.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"

namespace {
// This backend owns LEDC timer 0 and channels 0/1.
constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT;
// 1024 is valid at 10-bit resolution on C6 (below its maximum resolution).
constexpr std::uint32_t PWM_FULL_DUTY = 1U << 10;

struct Motor {
    gpio_num_t enable;
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
    bool inverted;
    bool pwm_configured = false;
    int direction = 0;
};

Motor left_motor{
    static_cast<gpio_num_t>(board::LEFT_MOTOR_ENABLE_PIN),
    static_cast<gpio_num_t>(board::LEFT_MOTOR_IN1_PIN),
    static_cast<gpio_num_t>(board::LEFT_MOTOR_IN2_PIN),
    LEDC_CHANNEL_0,
    board::LEFT_MOTOR_INVERTED,
};
Motor right_motor{
    static_cast<gpio_num_t>(board::RIGHT_MOTOR_ENABLE_PIN),
    static_cast<gpio_num_t>(board::RIGHT_MOTOR_IN1_PIN),
    static_cast<gpio_num_t>(board::RIGHT_MOTOR_IN2_PIN),
    LEDC_CHANNEL_1,
    board::RIGHT_MOTOR_INVERTED,
};

constexpr int MOTOR_PINS[]{
    board::LEFT_MOTOR_ENABLE_PIN, board::LEFT_MOTOR_IN1_PIN,
    board::LEFT_MOTOR_IN2_PIN, board::RIGHT_MOTOR_ENABLE_PIN,
    board::RIGHT_MOTOR_IN1_PIN, board::RIGHT_MOTOR_IN2_PIN,
};

constexpr bool valid_motor_pins()
{
    for (unsigned i = 0; i < 6; ++i) {
        const int pin = MOTOR_PINS[i];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pin) ||
            pin == board::MOSI_PIN || pin == board::MISO_PIN ||
            pin == board::SCLK_PIN || pin == board::IMU_CS_PIN ||
            pin == 12 || pin == 13) {  // Preserve the USB console.
            return false;
        }
        for (unsigned j = 0; j < i; ++j) {
            if (pin == MOTOR_PINS[j]) {
                return false;
            }
        }
    }
    return true;
}
static_assert(valid_motor_pins(), "Motor GPIOs must be valid, distinct and unused");

bool initialized = false;
motors::Config settings{};

esp_err_t disable_motor(Motor& motor)
{
    // ledc_stop forces EN low without waiting for a new duty value to take effect.
    const esp_err_t err = motor.pwm_configured
        ? ledc_stop(PWM_MODE, motor.channel, 0)
        : gpio_set_level(motor.enable, 0);
    if (err == ESP_OK) {
        motor.direction = 0;
    }
    return err;
}

esp_err_t disable_both()
{
    const esp_err_t left_err = disable_motor(left_motor);
    const esp_err_t right_err = disable_motor(right_motor);
    return left_err != ESP_OK ? left_err : right_err;
}

esp_err_t stop_on_error(esp_err_t err)
{
    // Attempt both stops even when one operation fails; retain the original error.
    (void)disable_both();
    return err;
}

esp_err_t configure_pwm(Motor& motor)
{
    ledc_channel_config_t channel{};
    channel.gpio_num = motor.enable;
    channel.speed_mode = PWM_MODE;
    channel.channel = motor.channel;
    channel.timer_sel = PWM_TIMER;
    channel.duty = 0;
    channel.hpoint = 0;
    const esp_err_t err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        return err;
    }
    motor.pwm_configured = true;
    return disable_motor(motor);
}

esp_err_t apply_duty(Motor& motor, float command)
{
    float duty = std::clamp(command, -settings.duty_limit, settings.duty_limit);
    if (motor.inverted) {
        duty = -duty;
    }
    const auto ticks = static_cast<std::uint32_t>(
        std::lround(std::abs(duty) * PWM_FULL_DUTY));
    if (ticks == 0) {
        return disable_motor(motor);
    }

    const int direction = duty > 0.0f ? 1 : -1;
    esp_err_t err;
    if (direction != motor.direction) {
        err = disable_motor(motor);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_set_level(motor.in1, direction > 0);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_set_level(motor.in2, direction < 0);
        if (err != ESP_OK) {
            return err;
        }
        motor.direction = direction;
    }

    err = ledc_set_duty(PWM_MODE, motor.channel, ticks);
    if (err != ESP_OK) {
        return err;
    }
    // update_duty also re-enables output after ledc_stop.
    return ledc_update_duty(PWM_MODE, motor.channel);
}
}  // namespace

esp_err_t motors::init(const Config& config)
{
    if (initialized) {
        return stop_on_error(ESP_ERR_INVALID_STATE);
    }
    if (config.pwm_frequency_hz == 0 || !std::isfinite(config.duty_limit) ||
        config.duty_limit <= 0.0f || config.duty_limit > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = disable_both();
    if (err != ESP_OK) {
        return stop_on_error(err);
    }
    gpio_config_t outputs{};
    for (const int pin : MOTOR_PINS) {
        // Preload low before enabling the GPIO output driver.
        err = gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        if (err != ESP_OK) {
            return stop_on_error(err);
        }
        outputs.pin_bit_mask |= 1ULL << pin;
    }
    outputs.mode = GPIO_MODE_OUTPUT;
    outputs.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs.pull_down_en = GPIO_PULLDOWN_ENABLE;
    outputs.intr_type = GPIO_INTR_DISABLE;
    err = gpio_config(&outputs);
    if (err != ESP_OK) {
        return stop_on_error(err);
    }

    ledc_timer_config_t timer{};
    timer.speed_mode = PWM_MODE;
    timer.duty_resolution = PWM_RESOLUTION;
    timer.timer_num = PWM_TIMER;
    timer.freq_hz = config.pwm_frequency_hz;
    timer.clk_cfg = LEDC_AUTO_CLK;
    err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return stop_on_error(err);
    }
    for (Motor* motor : {&left_motor, &right_motor}) {
        err = configure_pwm(*motor);
        if (err != ESP_OK) {
            return stop_on_error(err);
        }
    }

    settings = config;
    initialized = true;
    return ESP_OK;
}

esp_err_t motors::set_duty(float left, float right)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!std::isfinite(left) || !std::isfinite(right)) {
        return stop_on_error(ESP_ERR_INVALID_ARG);
    }
    esp_err_t err = apply_duty(left_motor, left);
    if (err == ESP_OK) {
        err = apply_duty(right_motor, right);
    }
    return err == ESP_OK ? ESP_OK : stop_on_error(err);
}

esp_err_t motors::stop()
{
    return initialized ? disable_both() : ESP_ERR_INVALID_STATE;
}
