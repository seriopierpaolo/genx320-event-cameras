#pragma once

#include "event_streamer.hpp"
#include "line_detector.hpp"
#include "servo_controller.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <cmath>

/**
 * @brief Renders a GenX320 event frame with detected Hough lines into an
 *        OpenCV window.
 *
 * Polarity convention:
 *   ON  (p == 1) → white  pixel
 *   OFF (p == 0) → grey   pixel
 * Background is black.
 * Detected lines are drawn in red; the line angle/rho are annotated.
 */
class Visualizer {
public:
    static constexpr int SENSOR_W = 320;
    static constexpr int SENSOR_H = 320;
    static constexpr int SCALE    = 3;   ///< display zoom factor

    explicit Visualizer(const std::string& window_name = "GenX320 Events")
        : win_(window_name)
        , canvas_(SENSOR_H * SCALE, SENSOR_W * SCALE, CV_8UC3)
    {
        cv::namedWindow(win_, cv::WINDOW_AUTOSIZE);
    }

    /**
     * @brief Render events + detected lines into the window.
     * @param events   Current event snapshot (only last window_us are shown).
     * @param lines    Hough lines to overlay.
     * @param frame_idx Frame counter shown in the title bar.
     */
    void show(const std::vector<Event>& events,
              const std::vector<DetectedLine>& lines,
              uint64_t frame_idx,
              uint64_t window_us,
              bool recording = false,
              int servo_us = SERVO_NEUTRAL)
    {
        canvas_.setTo(cv::Scalar(0, 0, 0));

        // ---- 1. Paint events -----------------------------------------------
        uint64_t t_max = 0;
        for (const auto& e : events)
            if (e.t > t_max) t_max = e.t;

        const uint64_t t_min = (window_us > 0 && t_max >= window_us)
                               ? (t_max - window_us) : 0;

        for (const auto& e : events) {
            if (e.t < t_min) continue;
            if (e.x >= SENSOR_W || e.y >= SENSOR_H) continue;

            const int px = e.x * SCALE;
            const int py = e.y * SCALE;

            const cv::Scalar colour = (e.p == 1)
                ? cv::Scalar(255, 255, 255)   // ON  → white
                : cv::Scalar(128, 128, 128);  // OFF → grey

            cv::rectangle(canvas_,
                          cv::Point(px, py),
                          cv::Point(px + SCALE - 1, py + SCALE - 1),
                          colour, cv::FILLED);
        }

        // // ---- 2. Draw Hough lines -------------------------------------------
        for (const auto& l : lines) {
            // Convert (rho, theta) → two points on the image border
            const double cos_t = std::cos(l.theta_rad);
            const double sin_t = std::sin(l.theta_rad);

            // Scale rho to display coordinates
            const double rho_d = l.rho * SCALE;

            // Two far-apart points along the line
            const double x0 = cos_t * rho_d;
            const double y0 = sin_t * rho_d;

            const cv::Point p1(
                static_cast<int>(x0 + 2000.0 * (-sin_t)),
                static_cast<int>(y0 + 2000.0 * ( cos_t)));
            const cv::Point p2(
                static_cast<int>(x0 - 2000.0 * (-sin_t)),
                static_cast<int>(y0 - 2000.0 * ( cos_t)));

            cv::line(canvas_, p1, p2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

            // Label: theta in degrees near the midpoint
            const cv::Point mid(canvas_.cols / 2, canvas_.rows / 2);
            const std::string lbl = cv::format("%.0f deg", l.theta_rad * 180.0 / M_PI);
            cv::putText(canvas_, lbl,
                        cv::Point(mid.x + static_cast<int>(rho_d * cos_t * 0.5),
                                  mid.y + static_cast<int>(rho_d * sin_t * 0.5)),
                        cv::FONT_HERSHEY_PLAIN, 0.9,
                        cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
        }

        // ---- 3. HUD --------------------------------------------------------
        const std::string hud = cv::format(
            "frame %llu | events %zu | lines %zu",
            static_cast<unsigned long long>(frame_idx),
            events.size(),
            lines.size());
        cv::putText(canvas_, hud, cv::Point(6, 16),
                    cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        // ---- 4. REC button -------------------------------------------------
        const cv::Rect btn = record_button();
        cv::rectangle(canvas_, btn,
                      recording ? cv::Scalar(0, 0, 200) : cv::Scalar(50, 50, 50),
                      cv::FILLED);
        cv::rectangle(canvas_, btn, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvas_, recording ? "\xe2\x97\x8f REC" : "REC",
                    cv::Point(btn.x + 6, btn.y + 15),
                    cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        // ---- 5. Servo direction arrow ----------------------------------------
        // servo_us < SERVO_NEUTRAL → motor commanded left  → arrow points left
        // servo_us > SERVO_NEUTRAL → motor commanded right → arrow points right
        {
            const int deviation = servo_us - SERVO_NEUTRAL;
            if (std::abs(deviation) > 5) {   // ignore tiny jitter
                const double max_dev = static_cast<double>(SERVO_MAX_US - SERVO_NEUTRAL); // 250
                const double ratio   = static_cast<double>(deviation) / max_dev;
                const int    arrow_dx = static_cast<int>(std::round(ratio * 80));  // ±80 px
                const cv::Point origin(canvas_.cols / 2, canvas_.rows / 2);
                const cv::Point tip(origin.x + arrow_dx, origin.y);
                cv::arrowedLine(canvas_, origin, tip,
                                cv::Scalar(0, 255, 255),   // yellow
                                3, cv::LINE_AA, 0, 0.3);
            }
        }

        cv::imshow(win_, canvas_);
        cv::waitKey(1);   // pump the GUI event loop
    }

    /// Call from the main thread after all threads have joined.
    void close() { cv::destroyWindow(win_); }

    /// Screen rectangle of the REC button (used by the mouse callback).
    static cv::Rect record_button() {
        return cv::Rect(SENSOR_W * SCALE - 80, 4, 74, 22);
    }

private:
    std::string win_;
    cv::Mat     canvas_;
};
