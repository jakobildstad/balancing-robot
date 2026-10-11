#pragma once

namespace console {

enum class Command { none, ping, test_left, test_right, balance, stop, disconnected, console_error };

// Configure nonblocking console input once at startup.
bool init();

// Read at most 16 characters per call, retaining partial commands until Enter.
// Reports disconnected separately so unplugged USB does not stop Wi-Fi control.
Command read_command();

}  // namespace console
