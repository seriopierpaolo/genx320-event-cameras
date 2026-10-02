#pragma once

#include "command_handler.hpp"
#include "event_streamer.hpp"

#include <string>
#include <thread>
#include <atomic>

/**
 * @brief Creates a virtual serial port via a PTY (pseudo-terminal) and
 *        dispatches incoming commands to the CommandHandler.
 *
 * Usage:
 *   Simulator sim("vertical_line.txt");
 *   sim.run();          // blocks until Ctrl-C / stop() is called
 */
class Simulator {
public:
    explicit Simulator(const std::string& events_file);
    ~Simulator();

    /// Open the PTY, print the slave path, and enter the read loop.
    void run();

    /// Signal the run-loop to exit (safe to call from signal handler).
    void stop();

    /// Path of the slave PTY device (available after run() is entered).
    std::string pty_path() const { return pty_path_; }

private:
    void read_loop();
    void write_response(const std::string& msg);

    int                pty_master_fd_{-1};
    int                pty_slave_fd_{-1};
    std::string        pty_path_;
    std::string        pty_symlink_;

    FirmwareState      state_;
    EventStreamer      streamer_;
    CommandHandler     handler_;

    std::atomic<bool>  running_{false};
};
