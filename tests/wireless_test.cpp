#include "wireless_protocol.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {
using console::Command;
using wireless::Request;
using wireless::RequestType;
using wireless::Session;

void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

constexpr Request START{RequestType::command, Command::balance};
constexpr Request STOP{RequestType::command, Command::stop};
constexpr Request HEARTBEAT{RequestType::heartbeat};

void parsing()
{
    Request request;
    for (const auto& text : {"balance", "stop", "test left", "test right"}) {
        check(wireless::parse_request(text, request), "Command rejected");
        check(request.type == RequestType::command, "Wrong command type");
        const auto expected = std::string(text) == "balance" ? Command::balance :
            std::string(text) == "stop" ? Command::stop :
            std::string(text) == "test left" ? Command::test_left : Command::test_right;
        check(request.command == expected, "Wrong parsed command");
    }
    check(wireless::parse_request("heartbeat", request) &&
          request.type == RequestType::heartbeat, "Heartbeat rejected");
    check(wireless::parse_request("gains 2.5 0.125", request) &&
          request.type == RequestType::gains && request.kp == 2.5f && request.kd == 0.125f,
          "Gains parsed incorrectly");
    check(wireless::parse_request("gains 0 0", request), "Zero gains rejected");
    check(wireless::parse_request("gains 1e-2 3e+1", request), "Scientific notation rejected");
    for (const auto& text : {"", "test", "balance extra", "stop\n", "gains", "gains ",
                             "gains 1", "gains 1 ", "gains 1 2 3", "gains 1 2x",
                             "gains -1 0", "gains 0 -1", "gains nan 0", "gains 0 inf",
                             "gains 1e100 0", "gains 0 1e-100", "gains x 2"}) {
        check(!wireless::parse_request(text, request), "Malformed request accepted");
    }
    check(!wireless::parse_request(std::string("gains 1 2\0junk", 14), request),
          "Embedded NUL accepted");
    check(!wireless::parse_request("gains " + std::string(64, '1') + " 0", request),
          "Oversized gains accepted");
}

void stop_priority()
{
    Session session;
    Request request;
    check(!session.receive(-1, START, 0), "Disconnected session accepted command");
    session.disconnect(-1);
    check(!session.take(request, 0), "Invalid disconnect generated event");
    check(session.connect(4, 0), "First connection rejected");
    check(!session.connect(5, 0), "Second owner accepted");
    check(!session.receive(5, STOP, 0), "Non-owner accepted");
    check(session.receive(4, START, 0), "Start rejected");
    check(!session.receive(4, START, 0), "Pending request overwritten");
    check(session.receive(4, STOP, 0), "Stop blocked by start");
    check(!session.receive(4, START, 0), "Start overwrote stop");
    check(session.receive(4, HEARTBEAT, 0), "Stop blocked heartbeat");
    check(session.take(request, 0) && request.command == Command::stop, "Stop not delivered first");
    check(!session.take(request, 0), "Stale start survived stop");
    check(session.receive(4, START, 0), "Explicit rearm rejected");
    check(session.take(request, 0) && request.command == Command::balance, "Rearm lost");
}

void timeout()
{
    Session session;
    Request request;
    session.connect(4, 100);
    check(session.connected(100 + Session::TIMEOUT_US), "Timeout boundary too early");
    check(session.receive(4, START, 100), "Start rejected");
    const auto late = 101 + Session::TIMEOUT_US;
    check(!session.connected(late), "Expired connection alive");
    check(session.take(request, late) && request.type == RequestType::link_lost,
          "Timeout did not signal link loss");
    check(!session.take(request, late), "Expired start survived timeout");
    check(!session.receive(4, START, late), "Expired connection accepted start");
    check(session.receive(4, STOP, late), "Timeout prevented stop");
    check(session.take(request, late) && request.command == Command::stop, "Stop lost");
    check(!session.receive(4, HEARTBEAT, late), "Late heartbeat revived expired connection");
    check(!session.connected(late), "Expired connection revived");
    check(!session.take(request, late), "Heartbeat auto-rearmed motion");

    session.disconnect(4);
    check(session.connect(6, late), "Fresh connection rejected");
    check(session.take(request, late) && request.type == RequestType::link_lost,
          "Reconnect lost disconnect event");
    check(session.receive(6, HEARTBEAT, late), "Fresh heartbeat rejected");
    check(session.receive(6, START, late), "Explicit rearm rejected");
    // A late heartbeat must trigger link loss even if main has not polled yet.
    const auto later = late + Session::TIMEOUT_US + 1;
    check(!session.receive(6, HEARTBEAT, later), "Buffered heartbeat revived connection");
    check(!session.receive(6, START, later), "Start bypassed pending link loss");
    check(session.take(request, later) && request.type == RequestType::link_lost,
          "Late heartbeat hid link loss");
    check(!session.take(request, later), "Late heartbeat preserved stale start");
    check(!session.receive(6, START, later), "Expired socket rearmed without reconnecting");
}

void reconnect()
{
    Session session;
    Request request;
    session.connect(4, 0);
    session.receive(4, START, 0);
    session.disconnect(5);
    check(session.connected(0), "Unrelated socket disconnected owner");
    session.disconnect(4);
    check(!session.connected(0), "Disconnected session alive");
    check(session.connect(6, 1), "Reconnect rejected");
    check(!session.receive(6, START, 1), "Reconnect bypassed pending link loss");
    check(session.take(request, 1) && request.type == RequestType::link_lost,
          "Reconnect hid disconnect event");
    check(!session.take(request, 1), "Reconnect replayed old command");
    check(session.receive(6, START, 1), "New explicit start rejected");
    check(session.take(request, 1) && request.command == Command::balance, "New start lost");
    session.receive(6, STOP, 1);
    session.disconnect(6);
    check(session.take(request, 1) && request.command == Command::stop,
          "Disconnect swallowed explicit stop");
}
}  // namespace

int main(int argc, char** argv)
{
    check(argc == 2, "Supply one test scenario");
    const std::string scenario = argv[1];
    if (scenario == "parsing") parsing();
    else if (scenario == "stop_priority") stop_priority();
    else if (scenario == "timeout") timeout();
    else if (scenario == "reconnect") reconnect();
    else check(false, "Unknown scenario");
}
