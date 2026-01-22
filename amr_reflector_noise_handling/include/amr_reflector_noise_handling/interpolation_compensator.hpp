#pragma once

#include <vector>
#include <algorithm>
#include "types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief Interpolation compensator for sparse point clouds
 * 
 * For clusters with >5 points and sparse distribution (5-10cm gaps),
 * interpolates additional points between adjacent points to improve
 * reflector detection at long distances.
 */
class InterpolationCompensator {
public:
    /**
     * @brief Constructor
     */
    InterpolationCompensator()
        : min_points_(5)
        , min_gap_(0.05)     // 5cm minimum gap
        , max_gap_(0.10)     // 10cm maximum gap
    {}
    
    /**
     * @brief Compensate sparse cluster by interpolation
     * @param cluster Cluster points to compensate
     * @return Compensated cluster with interpolated points
     */
    std::vector<Point> compensate(const std::vector<Point>& cluster) {
        if (cluster.size() < static_cast<size_t>(min_points_)) {
            // Too few points, skip compensation
            return cluster;
        }
        
        // Sort points by angle
        auto sorted = sortByAngle(cluster);
        
        std::vector<Point> compensated;
        compensated.reserve(sorted.size() * 2);  // Pre-allocate for possible interpolation
        
        // Process each adjacent pair
        for (size_t i = 0; i < sorted.size(); ++i) {
            compensated.push_back(sorted[i]);
            
            size_t next = (i + 1) % sorted.size();
            double dist = sorted[i].distanceTo(sorted[next]);
            
            // Check if interpolation is needed
            if (dist >= min_gap_ && dist <= max_gap_) {
                // Linear interpolation
                Point interpolated;
                interpolated.x = (sorted[i].x + sorted[next].x) / 2.0;
                interpolated.y = (sorted[i].y + sorted[next].y) / 2.0;
                interpolated.intensity = (sorted[i].intensity + sorted[next].intensity) / 2.0;
                
                compensated.push_back(interpolated);
            }
        }
        
        return compensated;
    }
    
    /**
     * @brief Set compensation parameters
     */
    void setMinPoints(int min_pts) { min_points_ = min_pts; }
    void setMinGap(double min_gap) { min_gap_ = min_gap; }
    void setMaxGap(double max_gap) { max_gap_ = max_gap; }

private:
    int min_points_;      // Minimum points required for compensation
    double min_gap_;      // Minimum gap to trigger interpolation (m)
    double max_gap_;      // Maximum gap to trigger interpolation (m)
    
    /**
     * @brief Sort points by angle from origin
     */
    std::vector<Point> sortByAngle(const std::vector<Point>& points) const {
        std::vector<Point> sorted = points;
        
        std::sort(sorted.begin(), sorted.end(),
            [](const Point& a, const Point& b) {
                return a.angleFromOrigin() < b.angleFromOrigin();
            });
        
        return sorted;
    }
};

} // namespace amr_reflector_noise_handling