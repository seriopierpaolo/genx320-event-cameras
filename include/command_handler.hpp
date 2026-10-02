#pragma once

#include "event_streamer.hpp"
#include <string>
#include <map>
#include <functional>

/// Holds the mutable firmware state (biases, LED, sensor on/off …).
struct FirmwareState {
    bool sensor_enabled{false};
    bool event_streaming{false};

    // Axes transforms
    bool flip_x{false};
    bool flip_y{false};
    bool swap_xy{false};
    bool first_only{false};

    // Event polarity filter: -1=all, 0=negative, 1=positive
    int polarity_filter{-1};

    // Temporal down-sampling: send every n-th event
    uint32_t downsample{1};

    // Biases: index -> value (GenX320 has ~11 biases, we store them generically)
    std::map<int, int> biases;

    // LED state: 0=off, 1=on, 2=blinking, 3=alarm
    int led_state{0};
    int led_period_ms{0};

    // Servo state
    bool servo_enabled{false};
    int  servo_position{1500};   ///< pulse width in µs [500, 2500]

    std::string fw_version{"v1.0.0-sim"};
    std::string chip_info{"GenX320 EVK Simulator | 320x240 | SN:SIM-0001"};
};

/**
 * @brief Parses a single command string and produces a response string.
 *
 * The handler also drives the EventStreamer (start/stop, filter updates)
 * according to the parsed command.
 */
class CommandHandler {
public:
    explicit CommandHandler(FirmwareState& state, EventStreamer& streamer, int pty_fd);

    /// Update the PTY file descriptor (called after the PTY is opened).
    void set_fd(int fd) { pty_fd_ = fd; }

    /**
     * @brief Process one line of input.
     * @param line Raw command string (newline already stripped).
     * @return Response to send back over the serial port (may be multi-line).
     */
    std::string process(const std::string& line);

private:
    // --- individual command handlers ---
    std::string cmd_sensor_enable(bool on);
    std::string cmd_event_stream(bool on);
    std::string cmd_hot_pixel(bool clear);
    std::string cmd_flip(char axis);
    std::string cmd_swap_xy(bool on);
    std::string cmd_first_only(bool on);
    std::string cmd_polarity_filter(char mode);
    std::string cmd_downsample(uint32_t n);
    std::string cmd_set_bias(int idx, int value);
    std::string cmd_restore_biases();
    std::string cmd_get_bias(const std::string& arg);
    std::string cmd_bias_names();
    std::string cmd_led(char mode, int period_ms);
    std::string cmd_servo_enable();
    std::string cmd_servo_position(int pulse_us);
    std::string cmd_reset();
    std::string cmd_bootloader();
    std::string cmd_fw_version();
    std::string cmd_chip_info();
    std::string cmd_help();

    FirmwareState& state_;
    EventStreamer& streamer_;
    int            pty_fd_;

    // Default bias table (index -> {name, default_value})
    struct BiasInfo { std::string name; int default_val; };
    static const std::map<int, BiasInfo> kBiasTable;
};
