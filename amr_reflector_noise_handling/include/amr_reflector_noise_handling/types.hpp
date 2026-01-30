#pragma once

#include <vector>
#include <cmath>
#include <string>
#include <deque>
#include <limits>

namespace amr_reflector_noise_handling {

/**
 * @brief Point structure with position and intensity
 * 
 * This is a common data structure used across all modules
 * for representing laser scan points with intensity information.
 */
struct Point {
    double x, y;       // Position coordinates (m)
    double intensity;     // Intensity value (0-1 after normalization)
    int origin_index; // LaserScan的原始索引
    
    Point() : x(0), y(0), intensity(0) {}
    Point(double x_, double y_, double i = 0) : x(x_), y(y_), intensity(i) {}
    
    /**
     * @brief Compute Euclidean distance to another point
     */
    double distanceTo(const Point& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    /**
     * @brief Compute squared distance to another point (faster than distanceTo)
     */
    double squaredDistanceTo(const Point& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return dx * dx + dy * dy;
    }
    
    /**
     * @brief Compute distance from origin
     */
    double distanceFromOrigin() const {
        return std::sqrt(x * x + y * y);
    }
    
    /**
     * @brief Compute angle from origin
     */
    double angleFromOrigin() const {
        return std::atan2(y, x);
    }
};

/**
 * @brief Detected reflector information
 */
struct DetectedReflector {
    Point center;                    // Center position (m)
    double diameter;                  // Estimated diameter (m)
    double confidence;                // Detection confidence (0-1)
    int point_count;                 // Number of points in cluster
    double mean_intensity;             // Mean intensity of points
    int idx;                            // idx of cluster
    std::string rejection_reason;      // Reason if rejected
    
    DetectedReflector()
        : diameter(0.0)
        , confidence(0.0)
        , point_count(0)
        , mean_intensity(0.0),idx(-1)
    {}
};

/**
 * @brief Tracked reflector with global ID and state management
 *
 * This structure represents a reflector being tracked globally across multiple frames.
 * It includes position filtering, uncertainty estimation, and state machine management.
 */
struct TrackedReflector {
    // === Basic Information ===
    int global_id;                          // Permanent global ID
    Point global_position;                  // World frame position (latest detection)
    Point filtered_position;                // EMA filtered position
    DetectedReflector latest_detection;     // Latest detection data
    
    // === State Management ===
    enum State {
        TENTATIVE,  // Newly created, waiting for continuity verification
        CONFIRMED,  // Confirmed as real reflector, ID is permanent
        INACTIVE    // Not detected for a while, but ID is retained
    } state;
    
    // === Continuity Detection (Sliding Window) ===
    struct DetectionRecord {
      int64_t timestamp; // Detection timestamp ns
      Point position;    // Detection position
      double confidence; // Detection confidence
    };
    std::deque<DetectionRecord> detection_history;  // Detection records in sliding window
    
    // === Position Filtering and Uncertainty ===
    double position_std_dev;                // Position standard deviation (uncertainty estimate)，用于确定真实反光柱匹配距离
    double position_variance;               // Position variance (for std_dev calculation)
    double avg_confidence;                  // Average confidence
    double accumulated_confidence;          // Accumulated confidence (for average calculation)
    
    // === Statistics ===
    int total_detection_count;              // Total detection count
    int consecutive_detections;             // Consecutive detection count
    int consecutive_misses;                 // Consecutive miss count
    int64_t last_detection_time;            // Last detection time
    int64_t first_detection_time;           // First detection time

    // === Geometric Properties ===
    double diameter;                         // Estimated diameter (EMA filtered)
    double diameter_std_dev;                 // Diameter standard deviation
    
    // === Visualization ===
    int visualization_color_id;              // Visualization color ID (fixed based on ID)
    
    TrackedReflector()
        : global_id(-1)
        , state(TENTATIVE)
        , position_std_dev(0.0)
        , position_variance(0.0)
        , avg_confidence(0.0)
        , accumulated_confidence(0.0)
        , total_detection_count(0)
        , consecutive_detections(0)
        , consecutive_misses(0)
        , last_detection_time(0)
        , first_detection_time(0)
        , diameter(0.0)
        , diameter_std_dev(0.0)
        , visualization_color_id(0)
    {}
};

} // namespace amr_reflector_noise_handling