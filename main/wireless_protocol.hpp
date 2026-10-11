#pragma once

#include "console.hpp"

#include <cstdint>
#include <string_view>

namespace wireless {

enum class RequestType { command, gains, heartbeat, link_lost };

struct Request {
    RequestType type = RequestType::command;
    console::Command command = console::Command::none;
    float kp = 0.0f;
    float kd = 0.0f;
};

bool parse_request(std::string_view text, Request& request);

// One browser owns the connection. The caller protects this state between tasks.
class Session {
public:
    static constexpr std::int64_t TIMEOUT_US = 1'000'000;

    bool connect(int socket, std::int64_t now_us);
    void disconnect(int socket);
    bool receive(int socket, const Request& request, std::int64_t now_us);
    bool take(Request& request, std::int64_t now_us);
    bool connected(std::int64_t now_us) const;

private:
    void expire(std::int64_t now_us);

    int socket_ = -1;
    std::int64_t heartbeat_us_ = 0;
    Request pending_{};
    bool has_pending_ = false;
    bool stop_pending_ = false;
    bool lost_pending_ = false;
    bool timed_out_ = false;
};

}  // namespace wireless
