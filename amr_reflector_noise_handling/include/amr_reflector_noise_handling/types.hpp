#pragma once

#include <vector>
#include <cmath>
#include <string>

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

} // namespace amr_reflector_noise_handling