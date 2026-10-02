#include "command_handler.hpp"

#include <sstream>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <iostream>

// ---------------------------------------------------------------------------
// Static bias table – GenX320 has 11 configurable biases
// ---------------------------------------------------------------------------

const std::map<int, CommandHandler::BiasInfo> CommandHandler::kBiasTable = {
    {0,  {"bias_diff_on",       115}},
    {1,  {"bias_diff_off",      75}},
    {2,  {"bias_diff",          57}},
    {3,  {"bias_fo",            30}},
    {4,  {"bias_hpf",           0}},
    {5,  {"bias_refr",          20}},
    {6,  {"bias_pr",            100}},
    {7,  {"bias_sum",           80}},
    {8,  {"bias_cas",           0}},
    {9,  {"bias_buf",           0}},
    {10, {"bias_bulk",          0}},
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommandHandler::CommandHandler(FirmwareState& state,
                               EventStreamer& streamer,
                               int            pty_fd)
    : state_(state), streamer_(streamer), pty_fd_(pty_fd)
{
    // Populate default biases
    for (auto& [idx, info] : kBiasTable)
        state_.biases[idx] = info.default_val;
}

// ---------------------------------------------------------------------------
// Main dispatch
// ---------------------------------------------------------------------------

std::string CommandHandler::process(const std::string& raw)
{
    // Trim whitespace
    auto trim = [](const std::string& s) -> std::string {
        size_t start = s.find_first_not_of(" \t\r\n");
        size_t end   = s.find_last_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        return s.substr(start, end - start + 1);
    };

    std::string line = trim(raw);
    if (line.empty()) return "";

    // -----------------------------------------------------------------------
    // Single-character commands
    // -----------------------------------------------------------------------
    if (line == "+")  return cmd_sensor_enable(true);
    if (line == "-")  return cmd_sensor_enable(false);
    if (line == "R")  return cmd_reset();
    if (line == "pRog") return cmd_bootloader();

    // -----------------------------------------------------------------------
    // Query commands  (?...)
    // -----------------------------------------------------------------------
    if (line == "?V")  return cmd_fw_version();
    if (line == "?C")  return cmd_chip_info();
    if (line == "??")  return cmd_help();
    if (line == "?BN") return cmd_bias_names();

    // ?B[i] or ?Ba
    if (line.size() >= 2 && line[0] == '?' && line[1] == 'B') {
        std::string arg = (line.size() > 2) ? line.substr(2) : "a";
        return cmd_get_bias(arg);
    }

    // -----------------------------------------------------------------------
    // Action commands  (!...)
    // -----------------------------------------------------------------------
    if (line.size() >= 2 && line[0] == '!') {
        std::string sub = line.substr(1);  // strip leading '!'

        // !E...  – event-related commands
        if (!sub.empty() && sub[0] == 'E') {
            std::string earg = sub.substr(1);  // everything after 'E'

            if (earg == "+") return cmd_event_stream(true);
            if (earg == "-") return cmd_event_stream(false);

            // !E*  hot-pixel
            if (earg == "*")  return cmd_hot_pixel(false);
            if (earg == "*-") return cmd_hot_pixel(true);

            // !EF[X,Y,B,N]  – flip axes
            if (earg.size() == 2 && earg[0] == 'F') {
                char axis = std::toupper((unsigned char)earg[1]);
                if (axis == 'X' || axis == 'Y' || axis == 'B' || axis == 'N')
                    return cmd_flip(axis);
            }
            // !EF[+,-]  – first-only filter (comes after EF letter check)
            if (earg == "F+") return cmd_first_only(true);
            if (earg == "F-") return cmd_first_only(false);

            // !ES[+/-]  – swap XY
            if (earg == "S+") return cmd_swap_xy(true);
            if (earg == "S-") return cmd_swap_xy(false);

            // !EP[+,-,A]  – polarity filter
            if (earg.size() == 2 && earg[0] == 'P') {
                char m = std::toupper((unsigned char)earg[1]);
                if (m == '+' || m == '-' || m == 'A')
                    return cmd_polarity_filter(m);
            }

            // !ED=n  – down-sampling
            if (earg.size() >= 3 && earg[0] == 'D' && earg[1] == '=') {
                try {
                    uint32_t n = static_cast<uint32_t>(std::stoul(earg.substr(2)));
                    return cmd_downsample(n);
                } catch (...) {
                    return "ERR: invalid argument for !ED\n";
                }
            }

            return "ERR: unknown !E command: " + line + "\n";
        }

        // !B<i>=<v>  – set bias
        if (!sub.empty() && sub[0] == 'B') {
            std::string barg = sub.substr(1);
            auto eq = barg.find('=');
            if (eq != std::string::npos) {
                try {
                    int idx = std::stoi(barg.substr(0, eq));
                    int val = std::stoi(barg.substr(eq + 1));
                    return cmd_set_bias(idx, val);
                } catch (...) {
                    return "ERR: invalid bias syntax (expected !B<i>=<v>)\n";
                }
            }

            // !BD  – restore defaults
            if (barg == "D") return cmd_restore_biases();

            return "ERR: unknown !B command: " + line + "\n";
        }

        // !S+  / !S=<pulse_us>  – servo motor
        if (!sub.empty() && sub[0] == 'S') {
            std::string sarg = sub.substr(1);

            if (sarg == "+") return cmd_servo_enable();

            if (sarg.size() >= 2 && sarg[0] == '=') {
                try {
                    int pulse_us = std::stoi(sarg.substr(1));
                    return cmd_servo_position(pulse_us);
                } catch (...) {
                    return "ERR: invalid argument for !S= (expected integer)\n";
                }
            }

            return "ERR: unknown !S command: " + line + "\n";
        }

        // !S+  / !S=<pulse_us>  – servo motor
        if (!sub.empty() && sub[0] == 'S') {
            std::string sarg = sub.substr(1);

            if (sarg == "+") return cmd_servo_enable();

            if (sarg.size() >= 2 && sarg[0] == '=') {
                try {
                    int pulse_us = std::stoi(sarg.substr(1));
                    return cmd_servo_position(pulse_us);
                } catch (...) {
                    return "ERR: invalid argument for !S= (expected integer)\n";
                }
            }

            return "ERR: unknown !S command: " + line + "\n";
        }

        // !L[-/+/.]  – LED
        if (!sub.empty() && sub[0] == 'L') {
            std::string larg = sub.substr(1);
            if (larg.empty()) return "ERR: missing LED argument\n";

            char mode = larg[0];
            int period_ms = 0;

            // optional [=t]
            auto eq = larg.find('=');
            if (eq != std::string::npos) {
                try {
                    period_ms = std::stoi(larg.substr(eq + 1));
                } catch (...) {
                    return "ERR: invalid LED period\n";
                }
            }

            if (mode == '-' || mode == '+' || mode == '.' || mode == 'a' || mode == 'A')
                return cmd_led(mode, period_ms);

            return "ERR: unknown LED mode '" + std::string(1, mode) + "'\n";
        }

        return "ERR: unknown command: " + line + "\n";
    }

    return "ERR: unrecognised command: " + line + "\n";
}

// ---------------------------------------------------------------------------
// Individual command implementations
// ---------------------------------------------------------------------------

std::string CommandHandler::cmd_sensor_enable(bool on)
{
    state_.sensor_enabled = on;
    if (on) {
        if (!streamer_.is_running()) {
            if (!streamer_.start(pty_fd_))
                return "ERR: sensor enabled but failed to start event streamer (check events file)\n";
            state_.event_streaming = true;
        }
        return "OK: sensor enabled – event streaming started\n";
    } else {
        if (streamer_.is_running()) {
            streamer_.stop();
            state_.event_streaming = false;
        }
        return "OK: sensor disabled – event streaming stopped\n";
    }
}

std::string CommandHandler::cmd_event_stream(bool on)
{
    if (on) {
        if (!state_.sensor_enabled)
            return "ERR: sensor is disabled – enable with '+' first\n";
        if (streamer_.is_running())
            return "OK: event streaming already active\n";
        if (!streamer_.start(pty_fd_))
            return "ERR: failed to start event streamer (check events file)\n";
        state_.event_streaming = true;
        return "OK: event streaming started\n";
    } else {
        streamer_.stop();
        state_.event_streaming = false;
        return "OK: event streaming stopped\n";
    }
}

std::string CommandHandler::cmd_hot_pixel(bool clear)
{
    if (clear)
        return "OK: hot-pixel map cleared\n";
    return "OK: hot-pixel detection triggered – blocked pixels refreshed\n";
}

std::string CommandHandler::cmd_flip(char axis)
{
    switch (axis) {
        case 'X':
            state_.flip_x = true;
            state_.flip_y = false;
            streamer_.set_flip_x(true);
            streamer_.set_flip_y(false);
            return "OK: flip X axis enabled\n";
        case 'Y':
            state_.flip_x = false;
            state_.flip_y = true;
            streamer_.set_flip_x(false);
            streamer_.set_flip_y(true);
            return "OK: flip Y axis enabled\n";
        case 'B':
            state_.flip_x = true;
            state_.flip_y = true;
            streamer_.set_flip_x(true);
            streamer_.set_flip_y(true);
            return "OK: flip both axes enabled\n";
        case 'N':
        default:
            state_.flip_x = false;
            state_.flip_y = false;
            streamer_.set_flip_x(false);
            streamer_.set_flip_y(false);
            return "OK: axis flip cleared (none)\n";
    }
}

std::string CommandHandler::cmd_swap_xy(bool on)
{
    state_.swap_xy = on;
    streamer_.set_swap_xy(on);
    return std::string("OK: X/Y address swap ") + (on ? "enabled" : "disabled") + "\n";
}

std::string CommandHandler::cmd_first_only(bool on)
{
    state_.first_only = on;
    streamer_.set_first_only(on);
    return std::string("OK: first-only filter ") + (on ? "enabled" : "disabled") + "\n";
}

std::string CommandHandler::cmd_polarity_filter(char mode)
{
    if (mode == '+') {
        state_.polarity_filter = 1;
        streamer_.set_polarity_filter(1);
        return "OK: polarity filter set to POSITIVE events only\n";
    } else if (mode == '-') {
        state_.polarity_filter = 0;
        streamer_.set_polarity_filter(0);
        return "OK: polarity filter set to NEGATIVE events only\n";
    } else {
        state_.polarity_filter = -1;
        streamer_.set_polarity_filter(-1);
        return "OK: polarity filter set to ALL events\n";
    }
}

std::string CommandHandler::cmd_downsample(uint32_t n)
{
    if (n < 1) n = 1;
    state_.downsample = n;
    streamer_.set_downsample(n);
    return "OK: temporal down-sampling set to every " + std::to_string(n) +
           (n == 1 ? " event (all events)\n" : " events\n");
}

std::string CommandHandler::cmd_set_bias(int idx, int value)
{
    auto it = kBiasTable.find(idx);
    if (it == kBiasTable.end())
        return "ERR: unknown bias index " + std::to_string(idx) + "\n";

    value = std::max(0, std::min(255, value));
    state_.biases[idx] = value;
    return "OK: " + it->second.name + " [" + std::to_string(idx) + "] = " +
           std::to_string(value) + "\n";
}

std::string CommandHandler::cmd_restore_biases()
{
    for (auto& [idx, info] : kBiasTable)
        state_.biases[idx] = info.default_val;
    return "OK: all biases restored to factory defaults\n";
}

std::string CommandHandler::cmd_get_bias(const std::string& arg)
{
    std::ostringstream oss;
    if (arg == "a" || arg == "A") {
        oss << "Biases:\n";
        for (auto& [idx, info] : kBiasTable) {
            int val = state_.biases.count(idx) ? state_.biases.at(idx) : info.default_val;
            oss << "  [" << idx << "] " << info.name
                << " = " << val << "\n";
        }
    } else {
        try {
            int idx = std::stoi(arg);
            auto it = kBiasTable.find(idx);
            if (it == kBiasTable.end())
                return "ERR: unknown bias index " + arg + "\n";
            int val = state_.biases.count(idx) ? state_.biases.at(idx) : it->second.default_val;
            oss << it->second.name << " [" << idx << "] = " << val << "\n";
        } catch (...) {
            return "ERR: invalid bias index '" + arg + "'\n";
        }
    }
    return oss.str();
}

std::string CommandHandler::cmd_bias_names()
{
    std::ostringstream oss;
    oss << "Bias index / name table:\n";
    for (auto& [idx, info] : kBiasTable)
        oss << "  [" << idx << "] " << info.name
            << "  (default: " << info.default_val << ")\n";
    return oss.str();
}

std::string CommandHandler::cmd_led(char mode, int period_ms)
{
    std::string desc;
    switch (mode) {
        case '-': state_.led_state = 0; desc = "OFF";       break;
        case '+': state_.led_state = 1; desc = "ON";        break;
        case '.': state_.led_state = 2;
                  state_.led_period_ms = period_ms > 0 ? period_ms : 500;
                  desc = "BLINKING (" + std::to_string(state_.led_period_ms) + " ms)";
                  break;
        case 'a':
        case 'A': state_.led_state = 3;
                  state_.led_period_ms = period_ms > 0 ? period_ms : 100;
                  desc = "ALARM (" + std::to_string(state_.led_period_ms) + " ms)";
                  break;
        default:  return "ERR: unknown LED mode\n";
    }
    return "OK: LED " + desc + "\n";
}

std::string CommandHandler::cmd_servo_enable()
{
    state_.servo_enabled = true;
    return "OK: servo enabled (position " + std::to_string(state_.servo_position) + " us)\n";
}

std::string CommandHandler::cmd_servo_position(int pulse_us)
{
    if (!state_.servo_enabled)
        return "ERR: servo is off – enable with '!S+' first\n";
    static constexpr int SERVO_MIN = 500;
    static constexpr int SERVO_MAX = 2500;
    if (pulse_us < SERVO_MIN || pulse_us > SERVO_MAX)
        return "ERR: pulse width out of range [" + std::to_string(SERVO_MIN) +
               ", " + std::to_string(SERVO_MAX) + "]\n";

    const int prev = state_.servo_position;
    state_.servo_position = pulse_us;

    if (pulse_us > prev)
        std::cout << "[Servo] going right (" << prev << " -> " << pulse_us << " us)\n";
    else if (pulse_us < prev)
        std::cout << "[Servo] going left  (" << prev << " -> " << pulse_us << " us)\n";

    return "OK: servo position set to " + std::to_string(pulse_us) + " us\n";
}

std::string CommandHandler::cmd_reset()
{
    streamer_.stop();
    state_ = FirmwareState{};
    for (auto& [idx, info] : kBiasTable)
        state_.biases[idx] = info.default_val;
    return "OK: board reset – all settings restored to defaults\n";
}

std::string CommandHandler::cmd_bootloader()
{
    streamer_.stop();
    state_.event_streaming = false;
    return "OK: entering bootloader mode… (simulator stub – no actual flash)\n";
}

std::string CommandHandler::cmd_fw_version()
{
    return "FW: " + state_.fw_version + "\n";
}

std::string CommandHandler::cmd_chip_info()
{
    return "CHIP: " + state_.chip_info + "\n";
}

std::string CommandHandler::cmd_help()
{
    return
        "Supported Commands:\n"
        "  +/-           - enable/disable sensor\n"
        "  !E+/-         - enable/disable event streaming\n"
        "  !E*[-]        - hot pixel detection / clear blocked pixels\n"
        "  !EF[X,Y,B,N]  - flip axes X, Y, Both, None (HW)\n"
        "  !ES[+/-]      - swap X/Y addresses (SW)\n"
        "  !EF[+,-]      - first-pixel-only filter\n"
        "  !EP[+,-,A]    - polarity filter: positive / negative / all\n"
        "  !ED=n         - temporal down-sampling: send every n-th event\n"
        "  !B<i>=<v>     - set bias i to value v\n"
        "  !BD           - restore default biases\n"
        "  ?B[i]         - retrieve bias i  (?Ba = all)\n"
        "  ?BN           - show bias index/name table\n"
        "  !L-/+/.[=t]   - LED off/on/blinking/alarm [period ms]\n"
        "  !S+           - enable servo motor\n"
        "  !S=<us>       - set servo pulse width [500..2500] us\n"
        "  R             - reset board\n"
        "  pRog          - enter bootloader\n"
        "  ?V            - display firmware version\n"
        "  ?C            - display chip info\n"
        "  ??            - display this help\n";
}
