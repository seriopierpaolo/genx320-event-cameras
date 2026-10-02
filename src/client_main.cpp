#include "event_buffer.hpp"
#include "line_detector.hpp"
#include "servo_controller.hpp"
#include "visualizer.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <csignal>
#include <cstring>
#include <stdexcept>

// POSIX serial
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static constexpr std::size_t BUFFER_CAPACITY = 20'000;  ///< max events in ring buffer
static constexpr int    LINE_PERIOD_MS    = 200;          ///< Line detector runs every N ms
static constexpr int    VIZ_PERIOD_MS     = 1000 / 30;    ///< Visualizer refresh ~33 ms (30 Hz)
// SERVO_MIN_US, SERVO_MAX_US, SERVO_KP, SERVO_NEUTRAL defined in servo_controller.hpp
static constexpr int    SAVE_WINDOW_S     = 10;           ///< Accumulate events for this many seconds before saving
static constexpr int    MAX_LINES_SHOWN   = 5;            ///< top-N lines to print
static constexpr uint64_t WINDOW_US       = 50'000;       ///< 50 ms event window

// ---------------------------------------------------------------------------
// Globals (for clean signal-based shutdown)
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_recording{false};           ///< true while events are being captured
static std::atomic<bool> g_rec_button_clicked{false};  ///< set by the OpenCV mouse callback

static void on_signal(int) { g_running.store(false); }

// ---------------------------------------------------------------------------
// Unbounded save buffer – collects every event for periodic disk dumps
// ---------------------------------------------------------------------------
struct SaveBuffer {
    std::vector<Event> events;
    std::mutex         mu;

    void push(const Event& e) {
        std::lock_guard<std::mutex> lk(mu);
        events.push_back(e);
    }

    /// Atomically take all accumulated events and leave the buffer empty.
    std::vector<Event> drain() {
        std::lock_guard<std::mutex> lk(mu);
        std::vector<Event> out;
        out.swap(events);
        return out;
    }
};

// ---------------------------------------------------------------------------
// Open + configure a serial / PTY port in raw mode
// ---------------------------------------------------------------------------
static int open_serial(const std::string& path)
{
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
        throw std::runtime_error("Cannot open port '" + path + "': " +
                                 strerror(errno));

    // Set blocking reads
    ::fcntl(fd, F_SETFL, 0);

    struct termios tio{};
    if (::tcgetattr(fd, &tio) < 0)
        throw std::runtime_error("tcgetattr failed");

    ::cfmakeraw(&tio);
    ::cfsetispeed(&tio, B115200);
    ::cfsetospeed(&tio, B115200);

    // Read returns as soon as ≥1 byte arrives, no timeout
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tio) < 0)
        throw std::runtime_error("tcsetattr failed");

    return fd;
}

// ---------------------------------------------------------------------------
// Send a command and eat the confirmation line (best-effort)
// ---------------------------------------------------------------------------
static void send_command(int fd, const std::string& cmd)
{
    std::string msg = cmd + "\r";
    ::write(fd, msg.c_str(), msg.size());
    std::cout << "[client] Sent: " << cmd << "\n";
}

// ---------------------------------------------------------------------------
// Thread 1 – Serial reader
//   Reads bytes from the serial port, assembles lines, parses "E t x y p"
//   events, and pushes them into the shared ring buffer.
//   Non-event lines (responses, confirmations) are printed to stdout.
// ---------------------------------------------------------------------------
static void reader_thread(int fd,
                          EventBuffer<BUFFER_CAPACITY>& buf,
                          SaveBuffer& save_buf)
{
    uint8_t raw[4];

    // Helper: blocking read of exactly 'len' bytes into dst.
    // Returns false on unrecoverable error.
    auto read_exact = [&](uint8_t* dst, int len) -> bool {
        int pos = 0;
        while (pos < len && g_running.load()) {
            ssize_t n = ::read(fd, dst + pos, len - pos);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                    continue;
                }
                std::cerr << "[reader] read error: " << strerror(errno) << "\n";
                g_running.store(false);
                return false;
            }
            if (n == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            pos += static_cast<int>(n);
        }
        return g_running.load();
    };

    while (g_running.load()) {
        // --- Scan byte-by-byte until a sync byte is found (B0: MSB set). ---
        // This transparently discards any interleaved non-event bytes
        // (e.g. command response text) without raising a framing error.
        do {
            if (!read_exact(raw, 1)) return;
        } while (!(raw[0] & 0x80));

        // --- Read the three remaining bytes of the word (B1, B2, B3). ---
        if (!read_exact(raw + 1, 3)) return;

        // Decode big-endian 32-bit word:
        //   bit 31     : sync (always 1)
        //   bits 30-19 : 12-bit timestamp (µs)
        //   bit 18     : polarity
        //   bits 17-9  : 9-bit x (0..319)
        //   bits 8-0   : 9-bit y (0..319)
        uint32_t word = (static_cast<uint32_t>(raw[0]) << 24)
                      | (static_cast<uint32_t>(raw[1]) << 16)
                      | (static_cast<uint32_t>(raw[2]) <<  8)
                      |  static_cast<uint32_t>(raw[3]);

        Event e{};
        e.t = (word >> 19) & 0xFFFu;
        e.p = (word >> 18) & 0x1u;
        e.x = (word >>  9) & 0x1FFu;
        e.y =  word        & 0x1FFu;
        buf.push(e);
        if (g_recording.load()) save_buf.push(e);
    }
}

// ---------------------------------------------------------------------------
// Thread 2 – Line detector
//   Wakes up every LINE_PERIOD_MS, snapshots the event buffer,
//   runs the line detector, and updates the shared lines vector.
// ---------------------------------------------------------------------------
static void line_detector_thread(EventBuffer<BUFFER_CAPACITY>& buf,
                                 std::vector<DetectedLine>& shared_lines,
                         std::chrono::steady_clock::time_point& shared_lines_time,
                         std::mutex& lines_mutex)
{
    LineDetector detector(320, 320,
                          /*theta_steps=*/180,
                          WINDOW_US);

    uint64_t line_idx = 0;

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(LINE_PERIOD_MS));

        auto events = buf.snapshot();
        if (events.empty()) continue;

        auto lines = detector.detect(events);

        {
            std::lock_guard<std::mutex> lk(lines_mutex);
            shared_lines      = lines;
            shared_lines_time = std::chrono::steady_clock::now();
        }

        std::cout << "\n--- Lines #" << line_idx++
                  << "  [" << events.size() << " events"
                  << ", " << lines.size() << " line(s) detected] ---\n";

        int shown = 0;
        for (const auto& l : lines) {
            if (shown++ >= MAX_LINES_SHOWN) break;
            std::cout << "  " << LineDetector::line_to_string(l) << "\n";
        }

        if (lines.empty())
            std::cout << "  (no lines detected)\n";

        std::cout << std::flush;
    }
}

// ---------------------------------------------------------------------------
// Thread 3 – Servo P-controller
//   Reads the latest detected line from shared_lines and drives the servo
//   to centre the line in the field of view:
//     line right of centre → servo decreases (camera pans left)
//     line left  of centre → servo increases (camera pans right)
// ---------------------------------------------------------------------------
static void servo_thread(int fd,
                          std::vector<DetectedLine>& shared_lines,
                          std::chrono::steady_clock::time_point& shared_lines_time,
                          std::mutex& lines_mutex,
                          std::atomic<int>& servo_pos_out)
{
    send_command(fd, "!S+");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int servo_pos = SERVO_NEUTRAL;
    servo_pos_out.store(servo_pos);
    send_command(fd, "!S=" + std::to_string(servo_pos));

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(LINE_PERIOD_MS));

        DetectedLine best{};
        bool has_line = false;
        {
            std::lock_guard<std::mutex> lk(lines_mutex);
            const auto age = std::chrono::steady_clock::now() - shared_lines_time;
            if (!shared_lines.empty() &&
                age < std::chrono::milliseconds(2 * LINE_PERIOD_MS)) {
                best     = shared_lines[0];   // highest-voted line
                has_line = true;
            }
        }

        if (!has_line) continue;

        servo_pos = compute_next_servo(best, servo_pos);
        servo_pos_out.store(servo_pos);

        send_command(fd, "!S=" + std::to_string(servo_pos));
    }
}

// ---------------------------------------------------------------------------
// OpenCV mouse callback – detects clicks on the REC button
// ---------------------------------------------------------------------------
static void on_mouse(int event, int x, int y, int /*flags*/, void* /*userdata*/)
{
    if (event != cv::EVENT_LBUTTONDOWN) return;
    if (Visualizer::record_button().contains(cv::Point(x, y)))
        g_rec_button_clicked.store(true);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n"
                  << "  e.g. " << argv[0] << " /dev/pts/3\n";
        return 1;
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string port_path = argv[1];

    int fd = -1;
    try {
        fd = open_serial(port_path);
    } catch (const std::exception& ex) {
        std::cerr << "[client] " << ex.what() << "\n";
        return 1;
    }

    std::cout << "[client] Connected to " << port_path << "\n";

    // Give the simulator a moment to send its banner, then drain it
    // using a temporary non-blocking flag so we don't block forever.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    {
        ::fcntl(fd, F_SETFL, O_NONBLOCK);
        char tmp[256];
        while (::read(fd, tmp, sizeof(tmp)) > 0) {}   // drain welcome banner
        ::fcntl(fd, F_SETFL, 0);                       // restore blocking
    }

    // Shared event ring buffer
    static EventBuffer<BUFFER_CAPACITY> event_buf;
    static SaveBuffer save_buf;

    // Create the OpenCV visualizer (must be done on the main thread on most platforms)
    Visualizer viz("GenX320 Events");
    cv::setMouseCallback("GenX320 Events", on_mouse);

    // Shared detected lines + timestamp, updated by the line detector thread and read by the viz loop
    std::vector<DetectedLine>             shared_lines;
    std::chrono::steady_clock::time_point shared_lines_time{};
    std::mutex lines_mutex;
    std::atomic<int> shared_servo_pos{SERVO_NEUTRAL};

    // Start reader thread
    std::thread reader(reader_thread, fd, std::ref(event_buf), std::ref(save_buf));

    // send_command(fd, "!E*D");
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Send start command: enable sensor (starts streaming)
    send_command(fd, "+");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Trigger hot-pixel filter, then let the sensor settle for 10 s
    // send_command(fd, "!E*");
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // Start servo P-controller thread
    std::thread servo(servo_thread, fd, std::ref(shared_lines),
                      std::ref(shared_lines_time), std::ref(lines_mutex),
                      std::ref(shared_servo_pos));

    // Start line detector thread
    std::thread line_det(line_detector_thread, std::ref(event_buf),
                         std::ref(shared_lines), std::ref(shared_lines_time),
                         std::ref(lines_mutex));

    std::cout << "[client] Running. Press Ctrl-C to stop.\n";

    // Visualization loop at 30 Hz on the main thread
    std::chrono::steady_clock::time_point record_start{};
    uint64_t frame_idx = 0;
    while (g_running.load()) {
        const auto t0 = std::chrono::steady_clock::now();

        // -- Handle REC button click --
        if (g_rec_button_clicked.exchange(false) && !g_recording.load()) {
            save_buf.drain();  // discard stale data from a previous session
            g_recording.store(true);
            record_start = std::chrono::steady_clock::now();
            std::cout << "[client] Recording started (" << SAVE_WINDOW_S << " s)...\n" << std::flush;
        }

        // -- Flush recording when the window has elapsed --
        if (g_recording.load()) {
            const auto rec_age = std::chrono::steady_clock::now() - record_start;
            if (rec_age >= std::chrono::seconds(SAVE_WINDOW_S)) {
                g_recording.store(false);
                auto saved = save_buf.drain();
                std::time_t now_t = std::time(nullptr);
                char ts[32];
                std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now_t));
                std::string filename = std::string("events_") + ts + ".txt";
                std::ofstream f(filename);
                if (f) {
                    for (const auto& e : saved)
                        f << e.t << " " << e.x << " " << e.y << " " << static_cast<int>(e.p) << "\n";
                    std::cout << "[client] Saved " << saved.size()
                              << " events to " << filename << "\n" << std::flush;
                } else {
                    std::cerr << "[client] Could not open '" << filename << "' for writing\n";
                }
            }
        }

        auto events = event_buf.snapshot();
        std::vector<DetectedLine> lines;
        {
            std::lock_guard<std::mutex> lk(lines_mutex);
            const auto age = std::chrono::steady_clock::now() - shared_lines_time;
            if (age < std::chrono::milliseconds(LINE_PERIOD_MS))
                lines = shared_lines;
        }
        viz.show(events, lines, frame_idx++, WINDOW_US, g_recording.load(),
                 shared_servo_pos.load());

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(VIZ_PERIOD_MS) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    reader.join();
    servo.join();
    line_det.join();

    // Send stop command
    send_command(fd, "-");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    viz.close();
    ::close(fd);
    std::cout << "[client] Done.\n";
    return 0;
}
