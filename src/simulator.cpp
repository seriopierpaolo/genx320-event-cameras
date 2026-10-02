#include "simulator.hpp"

#include <iostream>
#include <stdexcept>
#include <cstring>
#include <chrono>
#include <thread>

// POSIX / Linux PTY
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#ifdef __linux__
  #include <pty.h>
#else
  // macOS / BSD
  #include <util.h>
#endif

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

Simulator::Simulator(const std::string& events_file)
    : streamer_(events_file),
      handler_(state_, streamer_, -1)   // fd set to -1 until PTY is opened
{}

Simulator::~Simulator()
{
    stop();
    if (pty_slave_fd_ >= 0)
        ::close(pty_slave_fd_);
    if (pty_master_fd_ >= 0)
        ::close(pty_master_fd_);
    if (!pty_symlink_.empty())
        ::unlink(pty_symlink_.c_str());
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void Simulator::stop()
{
    running_.store(false);
    streamer_.stop();
}

void Simulator::run()
{
    // ------------------------------------------------------------------
    // Open a PTY master
    // ------------------------------------------------------------------
    pty_master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (pty_master_fd_ < 0)
        throw std::runtime_error(std::string("posix_openpt failed: ") + strerror(errno));

    if (::grantpt(pty_master_fd_) < 0)
        throw std::runtime_error(std::string("grantpt failed: ") + strerror(errno));

    if (::unlockpt(pty_master_fd_) < 0)
        throw std::runtime_error(std::string("unlockpt failed: ") + strerror(errno));

    char* slave_name = ::ptsname(pty_master_fd_);
    if (!slave_name)
        throw std::runtime_error(std::string("ptsname failed: ") + strerror(errno));

    pty_path_ = slave_name;

    // Open the slave side ourselves to keep the port live even when no
    // external client is connected (prevents EIO on the master).
    pty_slave_fd_ = ::open(slave_name, O_RDWR | O_NOCTTY);
    if (pty_slave_fd_ < 0)
        throw std::runtime_error(std::string("open slave PTY failed: ") + strerror(errno));

    // Create a stable symlink so other applications can find the port at
    // a predictable path regardless of which /dev/pts/N was allocated.
    pty_symlink_ = "/tmp/genx320-tty";
    ::unlink(pty_symlink_.c_str());   // remove stale symlink if any
    if (::symlink(slave_name, pty_symlink_.c_str()) < 0) {
        std::cerr << "[Simulator] Warning: could not create symlink "
                  << pty_symlink_ << ": " << strerror(errno) << "\n";
        pty_symlink_.clear();
    }

    // ------------------------------------------------------------------
    // Configure master side as raw (no echo, no line buffering)
    // ------------------------------------------------------------------
    struct termios tio{};
    if (::tcgetattr(pty_master_fd_, &tio) == 0) {
        ::cfmakeraw(&tio);
        ::tcsetattr(pty_master_fd_, TCSANOW, &tio);
    }

    // ------------------------------------------------------------------
    // Pass the real fd to the handler (and streamer indirectly)
    // ------------------------------------------------------------------
    handler_.set_fd(pty_master_fd_);

    const std::string& display_path = pty_symlink_.empty() ? pty_path_ : pty_symlink_;
    std::cout << "=======================================================\n"
              << " GenX320 Serial Port Simulator\n"
              << "=======================================================\n"
              << " Virtual serial port : " << pty_path_ << "\n";
    if (!pty_symlink_.empty())
        std::cout << " Stable symlink      : " << pty_symlink_ << "\n";
    std::cout << " Connect with        : screen " << display_path << " 115200\n"
              << "                       or minicom -D " << display_path << "\n"
              << " Send '?' '?' for help.\n"
              << "=======================================================\n";

    // Write a welcome banner to the slave side
    write_response(
        "\r\n"
        "GenX320 EVK Simulator – ready.\r\n"
        "Type '?''?' for command help.\r\n"
        "\r\n"
    );

    running_.store(true);
    read_loop();
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void Simulator::write_response(const std::string& msg)
{
    if (msg.empty()) return;

    // Replace bare '\n' with '\r\n' for terminals
    std::string out;
    out.reserve(msg.size() + 16);
    for (char c : msg) {
        if (c == '\n') out += '\r';
        out += c;
    }

    const char* p   = out.c_str();
    ssize_t     rem = static_cast<ssize_t>(out.size());
    while (rem > 0) {
        ssize_t n = ::write(pty_master_fd_, p, static_cast<size_t>(rem));
        if (n <= 0) break;
        p   += n;
        rem -= n;
    }
}

void Simulator::read_loop()
{
    std::string line_buf;
    char ch;

    while (running_.load()) {
        ssize_t n = ::read(pty_master_fd_, &ch, 1);

        if (n < 0) {
            if (errno == EINTR) continue;
            // EIO usually means no client connected (slave not open)
            // just keep polling
            if (errno == EIO) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            std::cerr << "[Simulator] read error: " << strerror(errno) << "\n";
            break;
        }

        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Echo back the character (local echo for terminal clients)
        ::write(pty_master_fd_, &ch, 1);

        if (ch == '\r' || ch == '\n') {
            if (!line_buf.empty()) {
                // Echo a newline
                write_response("\n");

                std::string response = handler_.process(line_buf);
                if (!response.empty())
                    write_response(response);

                line_buf.clear();
            }
        } else if (ch == 127 || ch == '\b') {
            // Backspace
            if (!line_buf.empty()) {
                line_buf.pop_back();
                // Erase character on terminal
                const char* bs = "\b \b";
                ::write(pty_master_fd_, bs, 3);
            }
        } else {
            line_buf += ch;
        }
    }
}
