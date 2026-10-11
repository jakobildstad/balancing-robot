#include "wireless_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>

bool wireless::parse_request(std::string_view text, Request& request)
{
    request = {};
    if (text == "heartbeat") {
        request.type = RequestType::heartbeat;
        return true;
    }
    if (text == "stop") request.command = console::Command::stop;
    else if (text == "balance") request.command = console::Command::balance;
    else if (text == "test left") request.command = console::Command::test_left;
    else if (text == "test right") request.command = console::Command::test_right;
    else if (text.substr(0, 6) == "gains " && text.size() < 64 &&
             text.find('\0') == std::string_view::npos) {
        char values[64]{};
        std::copy(text.begin() + 6, text.end(), values);
        char* end = nullptr;
        errno = 0;
        const float kp = std::strtof(values, &end);
        if (end == values || *end != ' ' || errno == ERANGE) return false;
        char* second = end + 1;
        const float kd = std::strtof(second, &end);
        if (end == second || *end != '\0' || errno == ERANGE ||
            !std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0f || kd < 0.0f) {
            return false;
        }
        request = {RequestType::gains, console::Command::none, kp, kd};
        return true;
    }
    return request.command != console::Command::none;
}

bool wireless::Session::connect(int socket, std::int64_t now_us)
{
    if (socket_ != -1 || socket < 0) return false;
    socket_ = socket;
    heartbeat_us_ = now_us;
    timed_out_ = false;
    return true;
}

void wireless::Session::disconnect(int socket)
{
    if (socket < 0 || socket != socket_) return;
    socket_ = -1;
    has_pending_ = false;
    lost_pending_ = true;
}

bool wireless::Session::connected(std::int64_t now_us) const
{
    return socket_ >= 0 && !timed_out_ && now_us >= heartbeat_us_ &&
           now_us - heartbeat_us_ <= TIMEOUT_US;
}

void wireless::Session::expire(std::int64_t now_us)
{
    if (socket_ >= 0 && !timed_out_ && !connected(now_us)) {
        timed_out_ = true;
        has_pending_ = false;
        lost_pending_ = true;
    }
}

bool wireless::Session::receive(int socket, const Request& request, std::int64_t now_us)
{
    if (socket < 0 || socket != socket_) return false;
    expire(now_us);
    if (request.type == RequestType::heartbeat) {
        // A timed-out connection must reconnect; buffered messages cannot rearm it.
        if (!connected(now_us)) return false;
        heartbeat_us_ = now_us;
        return true;
    }
    if (request.type == RequestType::command && request.command == console::Command::stop) {
        // Stop cannot be overwritten or held up by another pending request.
        stop_pending_ = true;
        has_pending_ = false;
        return true;
    }
    if (!connected(now_us) || stop_pending_ || lost_pending_ || has_pending_) return false;
    pending_ = request;
    has_pending_ = true;
    return true;
}

bool wireless::Session::take(Request& request, std::int64_t now_us)
{
    expire(now_us);
    if (stop_pending_) {
        stop_pending_ = false;
        lost_pending_ = false;
        request = {RequestType::command, console::Command::stop};
        return true;
    }
    if (lost_pending_) {
        lost_pending_ = false;
        request = {RequestType::link_lost};
        return true;
    }
    if (!has_pending_) return false;
    request = pending_;
    has_pending_ = false;
    return true;
}
