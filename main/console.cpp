#include "console.hpp"

#include <cerrno>
#include <cstddef>
#include <string_view>
#include <fcntl.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_stdio.h"

bool console::init()
{
    // ESP-IDF 6.1 nonblocking stdin reads need the USB driver's RX buffer.
    const esp_err_t err = esp_stdio_install_io_driver();
    if (err != ESP_OK) {
        ESP_LOGE("Console", "USB commands unavailable: %s",
                 esp_err_to_name(err));
        return false;
    }
    const int flags = fcntl(STDIN_FILENO, F_GETFL);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE("Console", "Cannot configure USB commands");
        return false;
    }
    ESP_LOGI("Motor", "Commands: ping, test left, test right, balance, stop (then Enter)");
    ESP_LOGI("Motor", "Tests: 300 ms at 100%%; 'test' aliases 'test left'. Balancing starts OFF");
    return true;
}

console::Command console::read_command()
{
    static char command[16]{};
    static std::size_t length = 0;
    static bool overflow = false;

    // Bound console work so input cannot starve the sensor/control loop.
    for (unsigned count = 0; count < sizeof(command); ++count) {
        char c;
        const ssize_t received = read(STDIN_FILENO, &c, 1);
        if (received <= 0) {
            if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                errno != EINTR && errno != EIO) {
                ESP_LOGE("Motor", "Console read failed: errno=%d", errno);
                return Command::console_error;
            }
            if (received < 0 && errno == EIO) {
                // Discard partial commands across USB disconnects.
                length = 0;
                overflow = false;
                return Command::disconnected;
            }
            return Command::none;
        }
        if (c != '\r' && c != '\n') {
            if (length < sizeof(command) - 1) {
                command[length++] = c;
            } else {
                overflow = true;
            }
            continue;
        }

        const std::string_view input{command, length};
        const bool too_long = overflow;
        length = 0;
        overflow = false;
        if (input.empty() && !too_long) {
            continue;
        }
        if (!too_long) {
            if (input == "ping") return Command::ping;
            if (input == "test" || input == "test left") return Command::test_left;
            if (input == "test right") return Command::test_right;
            if (input == "balance") return Command::balance;
            if (input == "stop") return Command::stop;
        }
        ESP_LOGW("Motor", "Use 'ping', 'test left', 'test right', 'balance' or 'stop'");
        return Command::none;
    }
    return Command::none;
}
