#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <vector>
#include <cstdint>

/// Represents a single DVS/ATIS event from a GenX320 sensor.
struct Event {
    uint64_t t;   ///< Timestamp in microseconds
    uint16_t x;   ///< Column  (0-319)
    uint16_t y;   ///< Row     (0-239)
    uint8_t  p;   ///< Polarity: 1 = ON, 0 = OFF
};

/**
 * @brief Loads events from a text file and streams them at real-time pace
 *        over an open file descriptor (the PTY master).
 *
 * File format (one event per line, '#' lines are comments):
 *   <t_us>  <x>  <y>  <p>
 *
 * Each event is written to the fd as the ASCII line:
 *   "E <t> <x> <y> <p>\n"
 *
 * The streamer loops through the file indefinitely until stopped.
 */
class EventStreamer {
public:
    explicit EventStreamer(const std::string& events_file);
    ~EventStreamer();

    /// Start streaming events to the given file descriptor.
    bool start(int fd);

    /// Stop streaming (blocks until the streaming thread exits).
    void stop();

    bool is_running() const { return running_.load(); }

    // --- Streaming filters (applied on the fly) ---
    void set_polarity_filter(int pol);   ///< -1 = all, 0 = OFF only, 1 = ON only
    void set_downsample(uint32_t n);     ///< send every n-th event (1 = all)
    void set_swap_xy(bool swap);
    void set_flip_x(bool flip);
    void set_flip_y(bool flip);
    void set_first_only(bool first);     ///< discard consecutive same-polarity events

private:
    void stream_loop(int fd);
    bool load_events();
    bool load_events_bin();   ///< binary camera format (32-bit words)
    bool load_events_txt();   ///< ASCII text format: <t_us> <x> <y> <p> per line
    bool validate_and_report(); ///< logs count + coordinate sanity check

    std::string           events_file_;
    std::vector<uint32_t> events_;          ///< raw 32-bit camera words
    std::vector<uint64_t> event_times_us_;  ///< full-precision timestamps for pacing (µs)

    std::thread        thread_;
    std::atomic<bool>  running_{false};

    // filter state (protected by mutex for lock-free reads from main thread)
    mutable std::mutex filter_mutex_;
    int      polarity_filter_{-1};  // -1=all
    uint32_t downsample_{1};
    bool     swap_xy_{false};
    bool     flip_x_{false};
    bool     flip_y_{false};
    bool     first_only_{false};

    // state for first_only filter
    int      last_polarity_{-1};
};
