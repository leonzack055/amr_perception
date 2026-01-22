#pragma once

#include "types.hpp"

#include <vector>
#include <cmath>
#include <algorithm>

namespace amr_reflector_noise_handling {

/**
 * @brief Intensity compensator for distance-dependent intensity variations
 * 
 * This class compensates for intensity variations caused by:
 * - Distance attenuation (inverse square law)
 * - Laser scanner characteristics
 * - Reflective properties at different angles
 */
class IntensityCompensator {
public:
    /**
     * @brief Constructor with default parameters
     */
    IntensityCompensator()
        : reference_distance_(2.0)
        , decay_exponent_(1.8)
        , angle_compensation_(true)
    {}
    
    /**
     * @brief Constructor with custom parameters
     * @param reference_distance Reference distance for compensation (m)
     * @param decay_exponent Decay exponent for distance compensation
     * @param angle_compensation Enable angle-based compensation
     */
    IntensityCompensator(double reference_distance, 
                       double decay_exponent,
                       bool angle_compensation = true)
        : reference_distance_(reference_distance)
        , decay_exponent_(decay_exponent)
        , angle_compensation_(angle_compensation)
    {}
    
    /**
     * @brief Compensate intensity for a single point
     * @param measured_intensity Measured intensity value
     * @param distance Distance to point (m)
     * @param angle Incident angle (rad, optional)
     * @return Compensated intensity value
     */
    double compensateIntensity(double measured_intensity, 
                          double distance,
                          double angle = 0.0) const {
        if (distance < 0.5) distance = 0.5; // Avoid division by zero
        
        // Distance compensation: I_corrected = I_measured * (d / d_ref)^2
        double distance_factor = std::pow(distance / reference_distance_, decay_exponent_);
        double compensated = measured_intensity * distance_factor;
        
        // Angle compensation (optional)
        if (angle_compensation_) {
            double angle_factor = compensateAngle(angle);
            compensated *= angle_factor;
        }
        
        return compensated;
    }
    
    /**
     * @brief Compensate intensity for a vector of points
     * @param points Vector of points with measured intensities
     * @return Vector of points with compensated intensities
     */
    std::vector<Point> compensateIntensities(const std::vector<Point>& points) const {
        std::vector<Point> compensated_points = points;
        
        for (auto& p : compensated_points) {
            double distance = std::sqrt(p.x * p.x + p.y * p.y);
            p.intensity = compensateIntensity(p.intensity, distance);
        }
        
        return compensated_points;
    }
    
    /**
     * @brief Normalize intensities to [0, 1] range
     * @param points Vector of points
     */
    static void normalizeIntensities(std::vector<Point>& points) {
        if (points.empty()) return;
        
        double max_intensity = 0.0;
        for (const auto& p : points) {
            if (p.intensity > max_intensity) {
                max_intensity = p.intensity;
            }
        }
        
        if (max_intensity > 1e-6) {
            for (auto& p : points) {
                p.intensity /= max_intensity;
            }
        }
    }
    
    /**
     * @brief Set reference distance
     */
    void setReferenceDistance(double distance) {
        reference_distance_ = distance;
    }
    
    /**
     * @brief Set decay exponent
     */
    void setDecayExponent(double exponent) {
        decay_exponent_ = exponent;
    }
    
    /**
     * @brief Enable or disable angle compensation
     */
    void setAngleCompensation(bool enable) {
        angle_compensation_ = enable;
    }

private:
    double reference_distance_;  // Reference distance (m)
    double decay_exponent_;   // Decay exponent for distance compensation
    bool angle_compensation_;  // Enable angle-based compensation
    
    /**
     * @brief Compensate for angle-dependent intensity
     * @param angle Incident angle (rad)
     * @return Angle compensation factor
     */
    double compensateAngle(double angle) const {
        // Cosine law: intensity proportional to cos(angle)
        // Add some tolerance for small angles
        double cos_angle = std::cos(angle);
        if (cos_angle < 0.3) cos_angle = 0.3; // Minimum compensation
        
        return 1.0 / cos_angle;
    }
};

} // namespace amr_reflector_noise_handling