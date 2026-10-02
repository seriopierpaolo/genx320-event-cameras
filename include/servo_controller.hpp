#pragma once

#include "line_detector.hpp"  // DetectedLine

#include <algorithm>          // std::clamp
#include <cmath>              // std::cos, std::sin, std::abs, std::round

#include <iostream>          // std::cout, std::endl

// ---------------------------------------------------------------------------
// Servo P-controller parameters
// ---------------------------------------------------------------------------

static constexpr double SERVO_KP        = 1.5;    ///< µs of servo correction per pixel of error
static constexpr int    SERVO_MIN_US    = 1000;   ///< Lower pulse-width bound (µs)
static constexpr int    SERVO_MAX_US    = 1500;   ///< Upper pulse-width bound (µs)
static constexpr int    SERVO_NEUTRAL   = (SERVO_MIN_US + SERVO_MAX_US) / 2;  ///< 1250 µs
static constexpr int    SENSOR_CENTER_X = 160;    ///< Target column for centering (sensor pixels)
static constexpr double SENSOR_H        = 320.0;  ///< Sensor height (pixels)

// ---------------------------------------------------------------------------
// ASSIGNMENT  –  implement compute_next_servo()
// ---------------------------------------------------------------------------
//
// Given the best detected line and the current servo pulse width, return the
// new pulse width that steers the camera toward the line.
//
// The servo is driven by a pulse-width signal in microseconds:
//   SERVO_MIN_US    – lower bound   (full left)
//   SERVO_MAX_US    – upper bound   (full right)
//   SERVO_NEUTRAL   – midpoint      (straight ahead)
//
// The return value must always be clamped to [SERVO_MIN_US, SERVO_MAX_US].
//
// You are free to choose any control strategy.  Some ideas:
//
//   1. Proportional (P) controller using line x-position:
//        – Convert (rho, theta) to an x-coordinate at the sensor's centre row:
//              x = (rho − (SENSOR_H/2)·sin(θ)) / cos(θ)
//        – error = x − SENSOR_CENTER_X   (positive → line is to the right)
//        – new_servo = clamp(current_servo − SERVO_KP × error, MIN, MAX)
//          (subtracting error makes the camera pan opposite to the line)
//
//   2. Bang-bang controller:
//        – If the line is to the right, jump straight to SERVO_MIN_US.
//        – If the line is to the left,  jump straight to SERVO_MAX_US.
//
//
// Parameters available above:
//   SERVO_KP        – suggested proportional gain (µs per pixel of error)
//   SERVO_MIN_US, SERVO_MAX_US, SERVO_NEUTRAL, SENSOR_CENTER_X, SENSOR_H
//
// ---------------------------------------------------------------------------

inline int compute_next_servo(const DetectedLine& line, int current_servo)
{
    // TODO: implement your servo control law here and return the new
    //       pulse width clamped to [SERVO_MIN_US, SERVO_MAX_US].
    // Find the x-coordinate of the line at the vertical midpoint of the sensor


      
    double x_pos = (line.rho + (SENSOR_CENTER_X));    //* std::sin(line.theta_rad)) / std::cos(line.theta_rad);

    

    std::cout << "Detected line at x = " << x_pos << " pixels from the center." << std::endl;

    if (x_pos < 0) {
        std::cout << "Line is to the left, turning right." << std::endl;
        return SERVO_MIN_US; // Line is to the left, turn right
    } else {
        std::cout << "Line is to the right, turning left." << std::endl;
        return SERVO_MAX_US; // Line is to the right, turn left
    }

    (void)line;
    //return current_servo;   // placeholder

    

    /*
    double cos_t = std::cos(line.theta_rad);
    
    // Safety guard for horizontal lines to avoid division by zero
    if (std::abs(cos_t) < 1e-5) {
        return current_servo;
    }

    // 1. Compute the true horizontal position of the line at the sensor's center row
    double x_pos = (line.rho - (SENSOR_H / 2.0) * std::sin(line.theta_rad)) / cos_t;
    
    // 2. Define the tracking error (Positive = line is to the right)
    double error = x_pos - SENSOR_CENTER_X;
    
    // Define the boundaries for our fuzzy sets (in pixels)
    // Adjust these if you want the "Zero/Centered" zone to be wider or narrower.
    const double target_range = 15.0;  // Inside ±15px is considered mostly centered
    const double max_range    = 60.0;  // Beyond ±60px is considered completely off-center

    // 3. Fuzzification: Calculate membership degrees μ in [0.0, 1.0]
    double mu_left = 0.0;
    double mu_zero = 0.0;
    double mu_right = 0.0;

    // Membership function for Left (Negative Error)
    if (error <= -max_range)        mu_left = 1.0;
    else if (error < -target_range) mu_left = (error + target_range) / (-max_range + target_range);

    // Membership function for Zero (Centered Error)
    if (error >= -target_range && error <= target_range) {
        mu_zero = 1.0;
    } else if (error > -max_range && error < -target_range) {
        mu_zero = (error + max_range) / (-target_range + max_range);
    } else if (error > target_range && error < max_range) {
        mu_zero = (max_range - error) / (max_range - target_range);
    }

    // Membership function for Right (Positive Error)
    if (error >= max_range)        mu_right = 1.0;
    else if (error > target_range) mu_right = (error - target_range) / (max_range - target_range);

    // 4. Rule Evaluation & Defuzzification (Centroid / Weighted Average Method)
    // Rule 1: If Left, then Turn Right (SERVO_MIN_US)
    // Rule 2: If Zero, then Stay Neutral (SERVO_NEUTRAL)
    // Rule 3: If Right, then Turn Left (SERVO_MAX_US)
    double sum_memberships = mu_left + mu_zero + mu_right;
    double target_servo = current_servo;

    if (sum_memberships > 0.0) {
        double weighted_output = (mu_left * SERVO_MIN_US) + 
                                 (mu_zero * SERVO_NEUTRAL) + 
                                 (mu_right * SERVO_MAX_US);
        target_servo = weighted_output / sum_memberships;
    }

    // 5. Output Clamping
    return std::clamp(static_cast<int>(std::round(target_servo)), SERVO_MIN_US, SERVO_MAX_US);
    */
}
