#pragma once

#include "event_streamer.hpp"   // Event struct

#include <vector>
#include <cstdint>
#include <string>

/// A single detected line in (rho, theta) parameterisation.
struct DetectedLine {
    double rho;        ///< Distance from origin (pixels)
    double theta_rad;  ///< Angle in radians  [0, π)
};

/**
 * @brief Event-based line detector.
 *
 * Receives a snapshot of events and returns the set of lines detected in
 * that snapshot, represented in (rho, theta) space and sorted by confidence
 * in descending order.
 *
 * The implementation is intentionally left as a student exercise — feel free
 * to replace the body of LineDetector::detect() with any algorithm you like.
 */
class LineDetector {
public:
    /**
     * @param sensor_w       Sensor width  (default 320)
     * @param sensor_h       Sensor height (default 320)
     * @param theta_steps    Hint for discretisation resolution (default 180)
     * @param window_us      Only consider events within this many µs of the
     *                       most recent event (0 = use all)
     */
    explicit LineDetector(int    sensor_w       = 320,
                           int    sensor_h       = 320,
                           int    theta_steps    = 180,
                           uint64_t window_us    = 50000);

    /**
     * @brief Run detection on a snapshot of events.
     * @return Detected lines sorted by confidence (descending).
     */
    std::vector<DetectedLine> detect(const std::vector<Event>& events);

    /// Human-readable summary of one line.
    static std::string line_to_string(const DetectedLine& l);

private:
    int      W_, H_;            ///< Sensor dimensions
    int      theta_steps_;      ///< Theta discretisation hint
    uint64_t window_us_;        ///< Time window for event filtering (0 = all events)

    // TODO: Add any additional private members your algorithm needs here.
};
