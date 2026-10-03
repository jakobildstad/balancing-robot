#pragma once

#include <cstdint>
#include "esp_err.h"

using ledc_mode_t = int;
using ledc_timer_t = int;
using ledc_timer_bit_t = int;
using ledc_channel_t = int;
constexpr ledc_mode_t LEDC_LOW_SPEED_MODE = 0;
constexpr ledc_timer_t LEDC_TIMER_0 = 0;
constexpr ledc_timer_bit_t LEDC_TIMER_10_BIT = 10;
constexpr ledc_channel_t LEDC_CHANNEL_0 = 0;
constexpr ledc_channel_t LEDC_CHANNEL_1 = 1;
constexpr int LEDC_AUTO_CLK = 0;

struct ledc_timer_config_t {
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    std::uint32_t freq_hz;
    int clk_cfg;
};

struct ledc_channel_config_t {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_timer_t timer_sel;
    std::uint32_t duty;
    int hpoint;
};

esp_err_t ledc_timer_config(const ledc_timer_config_t* config);
esp_err_t ledc_channel_config(const ledc_channel_config_t* config);
esp_err_t ledc_stop(ledc_mode_t mode, ledc_channel_t channel, std::uint32_t idle);
esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t channel, std::uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t channel);
