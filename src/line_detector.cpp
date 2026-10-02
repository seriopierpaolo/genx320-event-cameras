#include "line_detector.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <fstream>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LineDetector::LineDetector(int      sensor_w,
                             int      sensor_h,
                             int      theta_steps,
                             uint64_t window_us)
    : W_(sensor_w)
    , H_(sensor_h)
    , theta_steps_(theta_steps)
    , window_us_(window_us)
{
    // TODO: initialise any data structures your algorithm needs.
}

// ---------------------------------------------------------------------------
// Utility (provided)
// ---------------------------------------------------------------------------

std::string LineDetector::line_to_string(const DetectedLine& l)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1)
        << "rho=" << std::setw(7) << l.rho
        << " theta=" << std::setw(6) << (l.theta_rad * 180.0 / M_PI) << " deg";
    return oss.str();
}

// ---------------------------------------------------------------------------
// ASSIGNMENT  –  implement LineDetector::detect()
// ---------------------------------------------------------------------------
//
// Goal
// ----
// Given a snapshot of events from the GenX320 sensor, detect the line(s)
// present in the scene and return them as a list of DetectedLine structs
// sorted by confidence (highest first).
//
// Each DetectedLine carries:
//   rho       – perpendicular distance from the image origin to the line (px)
//   theta_rad – orientation of that perpendicular in [0, π)  (radians)
//
// The rest of the pipeline reads only lines[0] (the best line), so returning
// a single-element vector is perfectly fine.
// Constructor parameters available as member variables
// -----------------------------------------------------
//   W_, H_             – sensor width and height (320 × 320)
//   theta_steps_       – suggested angular resolution (180 bins → 1 °/bin)
//   window_us_         – consider only events within this many µs of the
//                        most recent event (0 = use all events)
//
// Suggested approaches
// --------------------
//  1. Hough Transform (classical batch approach)
//     a. Build a 2-D binary frame: mark every pixel that received at least
//        one event inside the time window.
//     b. For each active pixel (x, y), vote in a (rho, theta) accumulator:
//           rho = x·cos(θ) + y·sin(θ)   for each θ in [0, π)
//     c. Find accumulator peaks using non-maximum suppression, convert bin
//        indices back to (rho, theta) values, and return the peaks sorted by
//        accumulator count.
//

//




std::vector<DetectedLine> LineDetector::detect(const std::vector<Event>& events)
{
    if (events.empty()) return {};

    std::vector<DetectedLine> lines;


    /*
    std::vector<std::vector<int64_t>> image(H_, std::vector<int64_t>(W_, 0));

   
    for (const auto& ev : events) {

        
        image[ev.y][ev.x] += 1;
    }


    const double start = 0;
    const double end = 180;
    const double resolution = 1;
   
    size_t num_steps = static_cast<size_t>(std::ceil((end - start) / resolution)) + 1;

    std::vector<double> angles(num_steps);

  
    for (size_t i = 0; i < num_steps; ++i) {
        if (i == num_steps - 1) {
            angles[i] = end;
        } else {
            angles[i] = start + i * resolution;
        }
    }

    std::vector<std::vector<int64_t>> accumulator(theta_steps_, std::vector<int64_t>(rho, 0));
    for (int y = 0; y < H_; ++y) {
        for (int x = 0; x < W_; ++x) {
            if (image[y][x] > 0) { 
                for (const auto& theta : angles) {
                    double theta_rad = theta * M_PI / 180.0; // Convert to radians
                    double rho = x * std::cos(theta_rad) + y * std::sin(theta_rad);
                    // Vote in the accumulator (you need to define your accumulator structure)
                    accumulator[theta][rho] += 1; // Example voting
                }
            }
        }
    }



    for (size_t theta_idx = 0; theta_idx < accumulator.size(); ++theta_idx) {
        for (size_t rho_idx = 0; rho_idx < accumulator[theta_idx].size(); ++rho_idx) {
            int64_t votes = accumulator[theta_idx][rho_idx];
            if (votes > 0) {
                double theta_rad = angles[theta_idx] * M_PI / 180.0; // Convert back to radians
                double rho = rho;
                lines.push_back({rho, theta_rad});
            }
        }
    }    

    */


    // 1. Filter events by the time window (if window_us_ > 0)
    // Find the timestamp of the most recent event
    uint64_t max_ts = 0;
    for (const auto& ev : events) {
        if (ev.t > max_ts) max_ts = ev.t;
    }

    // 2. Build a 2D binary frame of active pixels within the window
    std::vector<std::vector<bool>> active_image(H_, std::vector<bool>(W_, false));
    for (const auto& ev : events) {
        if (window_us_ == 0 || (max_ts - ev.t) <= window_us_) {
            // Guard against out-of-bounds event data just in case
            if (ev.x >= 0 && ev.x < W_ && ev.y >= 0 && ev.y < H_) {
                active_image[ev.y][ev.x] = true;
            }
        }
    }

    int max_rho = static_cast<int>(std::ceil(std::sqrt(W_ * W_ + H_ * H_)));
  
    int rho_bins = 2 * max_rho + 1;

    std::vector<std::vector<int>> accumulator(theta_steps_, std::vector<int>(rho_bins, 0));

    std::vector<double> cos_table(theta_steps_);
    std::vector<double> sin_table(theta_steps_);
    for (int t = 0; t < theta_steps_; ++t) {
        double theta_rad = (static_cast<double>(t) / theta_steps_) * M_PI; // [0, π)
        cos_table[t] = std::cos(theta_rad);
        sin_table[t] = std::sin(theta_rad);
    }

    // 4. Vote in the accumulator
    for (int y = 0; y < H_; ++y) {
        for (int x = 0; x < W_; ++x) {
            if (active_image[y][x]) { 
                for (int t = 0; t < theta_steps_; ++t) {
                    double rho_val = x * cos_table[t] + y * sin_table[t];
                    // Map floating point rho_val to a valid integer bin index
                    int r_idx = static_cast<int>(std::round(rho_val)) + max_rho;
                    
                    if (r_idx >= 0 && r_idx < rho_bins) {
                        accumulator[t][r_idx]++;
                    }
                }
            }
        }
    }

    struct Peak {
        double rho;
        double theta_rad;
        int votes;
    };
    std::vector<Peak> peaks;

    for (int t = 0; t < theta_steps_; ++t) {
        for (int r = 0; r < rho_bins; ++r) {
            int votes = accumulator[t][r];
            if (votes > 150) {
                double rho_val = r - max_rho;
                double theta_val = (static_cast<double>(t) / theta_steps_) * M_PI;
                peaks.push_back({rho_val, theta_val, votes});
            }
        }
    }

    // 6. Sort peaks by confidence (votes) descending
    std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) {
        return a.votes > b.votes;
    });

    // 7. Convert to final output format
    
    for (const auto& p : peaks) {
        lines.push_back({p.rho, p.theta_rad});
    }


    
    // -----------------------------------------------------------------------
    // Visual Debugging: Save output image to a PPM file
    // -----------------------------------------------------------------------
    if (!lines.empty()) {
        std::string filename = "detected_line_output.ppm";
        std::ofstream ofs(filename, std::ios::binary);
        
        if (ofs.is_open()) {
            // Write standard PPM header (P6 means binary RGB, 255 is max color value)
            ofs << "P6\n" << W_ << " " << H_ << "\n255\n";

            // Grab the strongest detected line to draw
            const auto& best_line = lines[0];
            double r_val = best_line.rho;
            double t_rad = best_line.theta_rad;
            double cos_t = std::cos(t_rad);
            double sin_t = std::sin(t_rad);

            // Create a simple 1D buffer representing our RGB image
            std::vector<uint8_t> pixel_buffer(W_ * H_ * 3, 0);

            for (int y = 0; y < H_; ++y) {
                for (int x = 0; x < W_; ++x) {
                    int base_idx = (y * W_ + x) * 3;

                    // 1. Draw the underlying events in White
                    if (active_image[y][x]) {
                        pixel_buffer[base_idx + 0] = 255; // R
                        pixel_buffer[base_idx + 1] = 255; // G
                        pixel_buffer[base_idx + 2] = 255; // B
                    }

                    // 2. Overlay the best detected line in Red
                    // If the current pixel is within ~1.0 pixel distance of the line equation
                    double distance = std::abs(x * cos_t + y * sin_t - r_val);
                    if (distance < 1.0) {
                        pixel_buffer[base_idx + 0] = 255; // High Red
                        pixel_buffer[base_idx + 1] = 0;   // No Green
                        pixel_buffer[base_idx + 2] = 0;   // No Blue
                    }
                }
            }

            // Write the entire image buffer to disk
            ofs.write(reinterpret_cast<const char*>(pixel_buffer.data()), pixel_buffer.size());
            ofs.close();
        }
    }

   
    return lines;

    

    //return {};
}
