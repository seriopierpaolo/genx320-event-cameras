#pragma once

#include "event_streamer.hpp"   // re-uses the Event struct

#include <array>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <cstddef>

/**
 * @brief Thread-safe fixed-capacity circular buffer of Event objects.
 *
 * One thread (reader) calls push(); another (Hough) calls snapshot() to
 * obtain a copy of all currently stored events without blocking the reader
 * more than a brief mutex lock.
 *
 * @tparam N  Maximum number of events held at once.
 *            Older events are silently overwritten when the buffer is full.
 */
template<std::size_t N>
class EventBuffer {
public:
    /// Push a single event. Overwrites the oldest entry when full.
    void push(const Event& e) {
        std::lock_guard<std::mutex> lock(mu_);
        buf_[head_] = e;
        head_ = (head_ + 1) % N;
        if (count_ < N) ++count_;
        else tail_ = (tail_ + 1) % N;   // overwrite oldest
        cv_.notify_one();
    }

    /**
     * @brief Copy all stored events into a vector (oldest first).
     *        Returns immediately even if the buffer is empty.
     */
    std::vector<Event> snapshot() const {
        std::lock_guard<std::mutex> lock(mu_);
        std::vector<Event> out;
        out.reserve(count_);
        for (std::size_t i = 0; i < count_; ++i)
            out.push_back(buf_[(tail_ + i) % N]);
        return out;
    }

    /// Block until at least one event is available, then return snapshot.
    std::vector<Event> wait_and_snapshot() {
        std::unique_lock<std::mutex> lock(mu_);
        cv_.wait(lock, [this]{ return count_ > 0; });
        std::vector<Event> out;
        out.reserve(count_);
        for (std::size_t i = 0; i < count_; ++i)
            out.push_back(buf_[(tail_ + i) % N]);
        return out;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mu_);
        head_ = tail_ = count_ = 0;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mu_);
        return count_;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mu_);
        return count_ == 0;
    }

private:
    std::array<Event, N>   buf_{};
    std::size_t            head_{0};
    std::size_t            tail_{0};
    std::size_t            count_{0};
    mutable std::mutex     mu_;
    std::condition_variable cv_;
};
