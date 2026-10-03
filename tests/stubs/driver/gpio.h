#pragma once

#include <cstdint>
#include "esp_err.h"

using gpio_num_t = int;
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) ((pin) >= 0 && (pin) < 31)
constexpr int GPIO_MODE_OUTPUT = 2;
constexpr int GPIO_PULLUP_DISABLE = 0;
constexpr int GPIO_PULLDOWN_ENABLE = 1;
constexpr int GPIO_INTR_DISABLE = 0;

struct gpio_config_t {
    std::uint64_t pin_bit_mask;
    int mode;
    int pull_up_en;
    int pull_down_en;
    int intr_type;
};

esp_err_t gpio_config(const gpio_config_t* config);
esp_err_t gpio_set_level(gpio_num_t pin, std::uint32_t level);
