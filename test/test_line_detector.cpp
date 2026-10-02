/// test_line_detector.cpp
///
/// Tests:
///   1. LineDetector::detect() matches ground truth (data/ground_truth.txt)
///      within POSITION_TOLERANCE pixels for every 20 000-event chunk.
///   2. The P-controller driven by the ground truth median x positions moves
///      the servo in the opposite direction of the line (right → decrease,
///      left → increase), mirroring servo_thread() in client_main.cpp.
///   3. End-to-end: running the detector and feeding its output into the
///      same P-controller produces the correct servo direction for every chunk.

#include <gtest/gtest.h>
#include "servo_controller.hpp"   // compute_next_servo and all servo constants
                                   // (also pulls in line_detector.hpp → DetectedLine, LineDetector)

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// ===========================================================================
// Helpers
// ===========================================================================

struct GroundTruthEntry {
    uint64_t timestamp_us;
    int      median_x;
};

/// Load the two-column ground truth file (header lines start with '#').
static std::vector<GroundTruthEntry>
load_ground_truth(const std::string& path)
{
    std::vector<GroundTruthEntry> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        GroundTruthEntry e;
        if (iss >> e.timestamp_us >> e.median_x)
            out.push_back(e);
    }
    return out;
}

/// Load events from a text file (format: t x y p) in chunks of chunk_size.
/// Handles 12-bit timestamp wrap-arounds.
static std::vector<std::vector<Event>>
load_event_chunks(const std::string& path, int chunk_size = 20'000)
{
    std::vector<std::vector<Event>> chunks;
    std::vector<Event>              current;
    current.reserve(static_cast<size_t>(chunk_size));

    std::ifstream f(path);
    uint64_t wrap_epoch = 0;
    int      t_prev     = -1;
    std::string line;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        int t, x, y, p;
        if (std::sscanf(line.c_str(), "%d %d %d %d", &t, &x, &y, &p) != 4) continue;

        if (t_prev >= 0 && t < t_prev) wrap_epoch += 4096;
        t_prev = t;

        Event e;
        e.t = wrap_epoch + static_cast<uint64_t>(t);
        e.x = static_cast<uint16_t>(x);
        e.y = static_cast<uint16_t>(y);
        e.p = static_cast<uint8_t>(p);
        current.push_back(e);

        if (static_cast<int>(current.size()) == chunk_size) {
            chunks.push_back(std::move(current));
            current.clear();
            current.reserve(static_cast<size_t>(chunk_size));
        }
    }
    if (!current.empty())
        chunks.push_back(std::move(current));

    return chunks;
}

/// NOTE: compute_next_servo() comes directly from servo_controller.hpp –
///       same code used by servo_thread() in the client.

// ===========================================================================
// Test fixture
// ===========================================================================

class LineDetectorTest : public ::testing::Test {
protected:
    static constexpr int    CHUNK_SIZE         = 20'000;
    static constexpr int    SENSOR_CENTER_X    = 160;
    static constexpr double POSITION_TOLERANCE = 20.0;  // pixels
    // Skip direction check when the line is within this many pixels of centre
    // so that detection noise near the dead-band cannot flip the expected side.
    static constexpr int    DIRECTION_DEADBAND = static_cast<int>(POSITION_TOLERANCE) + 1;

    std::vector<std::vector<Event>> chunks_;
    std::vector<GroundTruthEntry>   gt_;
    // Detector: window_us=0 disables time-filtering since we pre-chunk by count.
    // Threshold and geometry match the live client.
    LineDetector detector_{320, 320, /*theta_steps=*/180, /*window_us=*/0};

    void SetUp() override
    {
        chunks_ = load_event_chunks("vertical_line.txt", CHUNK_SIZE);
        gt_     = load_ground_truth("ground_truth.txt");

        ASSERT_FALSE(chunks_.empty()) << "Could not load vertical_line.txt – "
                                         "run tests from the build directory";
        ASSERT_FALSE(gt_.empty())     << "Could not load ground_truth.txt – "
                                         "run tests from the build directory";
    }
};

// ===========================================================================
// Test 1 – Detected line position vs ground truth
// ===========================================================================

TEST_F(LineDetectorTest, DetectsVerticalLinePosition)
{
    const size_t n = std::min(chunks_.size(), gt_.size());

    for (size_t i = 0; i < n; ++i) {
        auto lines = detector_.detect(chunks_[i]);

        if (lines.empty()) {
            ADD_FAILURE() << "Chunk " << i << ": no lines detected";
            continue;
        }

        // Extract x from the detected line using the Hough formula for
        // comparison against ground truth (independent of the student’s
        // servo control implementation).
        const double cos_t     = std::cos(lines[0].theta_rad);
        const double detected_x = (std::abs(cos_t) < 1e-6)
            ? static_cast<double>(SENSOR_CENTER_X)
            : (lines[0].rho - (320.0 / 2.0) * std::sin(lines[0].theta_rad)) / cos_t;

        EXPECT_NEAR(detected_x, static_cast<double>(gt_[i].median_x),
                    POSITION_TOLERANCE)
            << "Chunk " << i << " (mid_t=" << gt_[i].timestamp_us << " µs)"
            << "  detected=" << detected_x
            << "  gt="       << gt_[i].median_x;
    }
}

// ===========================================================================
// Test 2 – P-controller driven by ground truth produces correct servo direction
// ===========================================================================

TEST_F(LineDetectorTest, GroundTruthImpliesCorrectServoDirection)
{
    for (size_t i = 0; i < gt_.size(); ++i) {
        // Construct a synthetic DetectedLine for a vertical line at the
        // ground truth column (theta=0, rho=x in Hough space) and ask the
        // student’s controller which direction to move.
        const DetectedLine gt_line{static_cast<double>(gt_[i].median_x), 0.0};
        const int next = compute_next_servo(gt_line, SERVO_NEUTRAL);

        if (gt_[i].median_x > SENSOR_CENTER_X + DIRECTION_DEADBAND) {
            // Line is on the right → servo must decrease (camera pans left)
            EXPECT_LT(next, SERVO_NEUTRAL)
                << "Entry " << i << ": median_x=" << gt_[i].median_x
                << " is right of centre – expected servo < neutral (" << SERVO_NEUTRAL << ")";
        } else if (gt_[i].median_x < SENSOR_CENTER_X - DIRECTION_DEADBAND) {
            // Line is on the left → servo must increase (camera pans right)
            EXPECT_GT(next, SERVO_NEUTRAL)
                << "Entry " << i << ": median_x=" << gt_[i].median_x
                << " is left of centre – expected servo > neutral (" << SERVO_NEUTRAL << ")";
        }
        // Near centre: no direction requirement.
    }
}

// ===========================================================================
// Test 3 – End-to-end: detector output → P-controller → correct servo direction
// ===========================================================================

TEST_F(LineDetectorTest, EndToEndServoDirectionMatchesGroundTruth)
{
    const size_t n = std::min(chunks_.size(), gt_.size());

    int chunks_with_detection = 0;

    for (size_t i = 0; i < n; ++i) {
        auto lines = detector_.detect(chunks_[i]);
        if (lines.empty()) continue;

        ++chunks_with_detection;

        const int gt_x       = gt_[i].median_x;
        const int next_servo = compute_next_servo(lines[0], SERVO_NEUTRAL);

        // Only assert direction when ground truth is clearly away from centre
        // (beyond the detection tolerance) so a small detection error cannot
        // flip the expected side.
        if (gt_x > SENSOR_CENTER_X + DIRECTION_DEADBAND) {
            EXPECT_LT(next_servo, SERVO_NEUTRAL)
                << "Chunk " << i
                << ": gt_x=" << gt_x
                << " – line is right of centre, servo should decrease (go left)";
        } else if (gt_x < SENSOR_CENTER_X - DIRECTION_DEADBAND) {
            EXPECT_GT(next_servo, SERVO_NEUTRAL)
                << "Chunk " << i
                << ": gt_x=" << gt_x
                << " – line is left of centre, servo should increase (go right)";
        }
    }

    EXPECT_GT(chunks_with_detection, 0)
        << "No lines were detected in any of the " << n << " chunks – "
           "check your LineDetector::detect() implementation";
}
