#include "event_streamer.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <unistd.h>   // write()

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

EventStreamer::EventStreamer(const std::string& events_file)
    : events_file_(events_file)
{}

EventStreamer::~EventStreamer()
{
    stop();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool EventStreamer::start(int fd)
{
    if (running_.load()) return false;

    if (events_.empty()) {
        if (!load_events_txt()) {
            std::cerr << "[EventStreamer] Failed to load events from '"
                      << events_file_ << "'\n";
            return false;
        }
    }

    running_.store(true);
    last_polarity_ = -1;
    thread_ = std::thread(&EventStreamer::stream_loop, this, fd);
    return true;
}

void EventStreamer::stop()
{
    running_.store(false);
    if (thread_.joinable())
        thread_.join();
}

// ---------------------------------------------------------------------------
// Filter setters (thread-safe)
// ---------------------------------------------------------------------------

void EventStreamer::set_polarity_filter(int pol)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    polarity_filter_ = pol;
}

void EventStreamer::set_downsample(uint32_t n)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    downsample_ = (n < 1) ? 1 : n;
}

void EventStreamer::set_swap_xy(bool swap)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    swap_xy_ = swap;
}

void EventStreamer::set_flip_x(bool flip)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    flip_x_ = flip;
}

void EventStreamer::set_flip_y(bool flip)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    flip_y_ = flip;
}

void EventStreamer::set_first_only(bool first)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    first_only_ = first;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool EventStreamer::load_events()
{
    // Auto-detect format by peeking at the first byte:
    //   binary : B0 always has MSB set (>= 0x80)
    //   text   : starts with ASCII digits, '#', or whitespace (< 0x80)
    std::ifstream probe(events_file_, std::ios::binary);
    if (!probe.is_open()) {
        std::cerr << "[EventStreamer] Failed to open events file: " << events_file_ << "\n";
        return false;
    }
    int first = probe.peek();
    if (first == EOF) {
        std::cerr << "[EventStreamer] Events file is empty: " << events_file_ << "\n";
        return false;
    }
    probe.close();

    if (static_cast<uint8_t>(first) < 0x80) {
        std::cout << "[EventStreamer] Detected text format.\n";
        return load_events_txt();
    }
    std::cout << "[EventStreamer] Detected binary format.\n";
    return load_events_bin();
}

bool EventStreamer::load_events_bin()
{
    std::ifstream file(events_file_, std::ios::binary);
    if (!file.is_open()) return false;

    // Scan byte-by-byte to find the first valid B0 (sync bit = MSB set).
    // This skips any incomplete leading word at the start of the file.
    uint8_t b;
    while (file.read(reinterpret_cast<char*>(&b), 1)) {
        if (b & 0x80) break;
    }
    if (!file.good()) {
        std::cerr << "[EventStreamer] No valid sync byte found in '" << events_file_ << "'\n";
        return false;
    }

    // Read the 3 remaining bytes of the first word, then continue 4 at a time.
    // A trailing partial word (< 4 bytes) is silently discarded by file.read.
    uint8_t buf[4];
    buf[0] = b;
    if (!file.read(reinterpret_cast<char*>(buf + 1), 3)) {
        std::cerr << "[EventStreamer] File too short after sync byte\n";
        return false;
    }

    uint64_t wrap_epoch = 0;

    do {
        uint32_t word = (static_cast<uint32_t>(buf[0]) << 24)
                      | (static_cast<uint32_t>(buf[1]) << 16)
                      | (static_cast<uint32_t>(buf[2]) <<  8)
                      |  static_cast<uint32_t>(buf[3]);

        if (word & (1u << 31)) {
            events_.push_back(word);

            // Reconstruct full monotonic timestamp from 12-bit value + wrap tracking
            const uint64_t ts_raw = (word >> 19) & 0xFFFu;
            if (!event_times_us_.empty() && ts_raw < (event_times_us_.back() & 0xFFFu))
                wrap_epoch += 4096u;
            event_times_us_.push_back(wrap_epoch + ts_raw);
        }

    } while (file.read(reinterpret_cast<char*>(buf), 4));

    return validate_and_report();
}

bool EventStreamer::load_events_txt()
{
    std::ifstream file(events_file_);
    if (!file.is_open()) return false;

    uint64_t wrap_epoch = 0;
    uint64_t t_prev     = 0;
    bool     first      = true;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        uint64_t t; uint16_t x, y; uint32_t p;
        if (!(iss >> t >> x >> y >> p)) continue;

        // Detect 12-bit (4096 µs) wrap-arounds: works for both raw-12-bit files
        // (vertical_line.txt, t in [0,4095]) and full-µs files (sample_events.txt,
        // t always increasing).  Full-µs timestamps never decrease so no epoch is
        // added for them.
        if (!first && t < t_prev) wrap_epoch += 4096u;
        t_prev = t;
        first  = false;

        // Encode into the 32-bit camera word format:
        //   bit 31     : sync (always 1)
        //   bits 30-19 : 12-bit timestamp (µs, lower 12 bits)
        //   bit 18     : polarity
        //   bits 17-9  : 9-bit x (0..319)
        //   bits 8-0   : 9-bit y (0..319)
        uint32_t word = (1u                    << 31)
                      | ((t & 0xFFFu)          << 19)
                      | ((p & 0x1u)            << 18)
                      | ((x & 0x1FFu)          <<  9)
                      |  (y & 0x1FFu);
        events_.push_back(word);
        event_times_us_.push_back(wrap_epoch + t);  // monotonic µs for pacing
    }

    return validate_and_report();
}

bool EventStreamer::validate_and_report()
{
    std::cout << "[EventStreamer] Loaded " << events_.size()
              << " events from '" << events_file_ << "'\n";

    // Sanity-check the first 10 000 events: x and y must be within [0, 319]
    const size_t check_count = std::min(events_.size(), size_t(10000));
    size_t invalid = 0;
    for (size_t i = 0; i < check_count; ++i) {
        uint16_t x = (events_[i] >>  9) & 0x1FFu;
        uint16_t y =  events_[i]        & 0x1FFu;
        if (x >= 320 || y >= 320) ++invalid;
    }
    if (invalid > 0)
        std::cerr << "[EventStreamer] WARNING: " << invalid << "/" << check_count
                  << " events in the first " << check_count
                  << " have out-of-bounds coordinates (x or y >= 320)"
                  << " – check data format.\n";
    else
        std::cout << "[EventStreamer] Validation OK: first " << check_count
                  << " events all within [0,319].\n";

    return !events_.empty();
}

void EventStreamer::stream_loop(int fd)
{
    if (events_.empty()) { running_.store(false); return; }

    auto           loop_start    = std::chrono::steady_clock::now();
    const uint64_t t0_us         = event_times_us_.front();
    uint32_t       event_counter = 0;

    while (running_.load()) {
        for (size_t i = 0; i < events_.size() && running_.load(); ++i) {
            const uint32_t raw    = events_[i];
            const uint64_t dt_us  = event_times_us_[i] - t0_us;
            const uint8_t  pol    = (raw >> 18) & 0x1u;

            // --- filters + optional spatial transforms (single lock) ---
            uint32_t word;
            {
                std::lock_guard<std::mutex> lock(filter_mutex_);

                if (polarity_filter_ != -1 && pol != (uint8_t)polarity_filter_)
                    continue;

                ++event_counter;
                if ((event_counter % downsample_) != 0)
                    continue;

                if (first_only_) {
                    if ((int)pol == last_polarity_) continue;
                    last_polarity_ = (int)pol;
                }

                // Spatial transforms: manipulate bit fields inline
                if (swap_xy_ || flip_x_ || flip_y_) {
                    uint16_t x = (raw >>  9) & 0x1FFu;
                    uint16_t y =  raw        & 0x1FFu;
                    if (flip_x_) x = 319 - x;
                    if (flip_y_) y = 319 - y;
                    if (swap_xy_) std::swap(x, y);
                    word = (raw & 0xFFFC0000u)  // keep sync + ts + polarity
                         | ((x & 0x1FFu) << 9)
                         |  (y & 0x1FFu);
                } else {
                    word = raw;
                }
            }

            // --- real-time pacing ---
            const auto desired_wall = loop_start + std::chrono::microseconds(dt_us);
            const auto now = std::chrono::steady_clock::now();
            if (desired_wall > now)
                std::this_thread::sleep_until(desired_wall);

            if (!running_.load()) break;

            // --- stream raw 4 bytes big-endian, no re-encoding needed ---
            uint8_t bytes[4] = {
                static_cast<uint8_t>(word >> 24),
                static_cast<uint8_t>(word >> 16),
                static_cast<uint8_t>(word >>  8),
                static_cast<uint8_t>(word)
            };
            ssize_t written = write(fd, bytes, 4);
            if (written < 0) {
                // PTY closed – stop silently
                running_.store(false);
                return;
            }
        }
        // Loop back: reset wall clock so the next pass is paced from now
        loop_start = std::chrono::steady_clock::now();
    }
}
