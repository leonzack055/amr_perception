#pragma once

#include <cmath>
#include <algorithm>

namespace amr_reflector_noise_handling {

/**
 * @brief Distance-adaptive parameters for reflective post detection
 * 
 * These parameters are adjusted based on the distance to the reflector
 * to optimize detection performance at different ranges.
 */
struct DistanceAdaptiveParams {
    double eps;              // Clustering neighborhood radius (m)
    int min_points;         // Minimum points for a valid cluster
    double intensity_thresh;  // Intensity threshold
    double diameter_min;      // Minimum acceptable diameter (m)
    double diameter_max;      // Maximum acceptable diameter (m)
    int max_points;         // Maximum points (for sampling)
    double arc_threshold;    // Arc threshold for filtering
};

/**
 * @brief Compute adaptive parameters based on distance to reflector
 * 
 * @param distance Distance from laser scanner to reflector (m)
 * @return DistanceAdaptiveParams Adaptive parameters for the given distance
 */
inline DistanceAdaptiveParams computeAdaptiveParams(double distance) {
    DistanceAdaptiveParams params;
    
    // Distance分段：近、中、远 (针对7cm直径反光柱优化)
    if (distance < 2.0) {
        // 近距离：点数多，需要更严格的过滤
        params.eps = 0.05;
        params.min_points = 8;
        params.intensity_thresh = 1500;
        params.diameter_min = 0.05;
        params.diameter_max = 0.20;
        params.max_points = 20;
        params.arc_threshold = 0.08;
    } else if (distance < 5.0) {
        // 中距离：标准参数 (适配7cm直径)
        params.eps = 0.065;
        params.min_points = 5;
        params.intensity_thresh = 1000;
        params.diameter_min = 0.06;
        params.diameter_max = 0.15;
        params.max_points = 15;
        params.arc_threshold = 0.1;
    } else {
        // 远距离：点数少，放宽条件
        params.eps = 0.08;
        params.min_points = 3;
        params.intensity_thresh = 600;
        params.diameter_min = 0.05;
        params.diameter_max = 0.25;
        params.max_points = 10;
        params.arc_threshold = 0.12;
    }
    
    return params;
}

/**
 * @brief Smooth parameter transitions to avoid sudden changes
 */
struct SmoothedParams {
    DistanceAdaptiveParams params;
    double alpha = 0.7; // Smoothing coefficient (0-1)
    
    /**
     * @brief Update parameters with smoothing
     * @param new_params New parameters to apply
     */
    void update(const DistanceAdaptiveParams& new_params) {
        params.eps = alpha * params.eps + (1 - alpha) * new_params.eps;
        params.min_points = static_cast<int>(
            alpha * params.min_points + (1 - alpha) * new_params.min_points);
        params.intensity_thresh = alpha * params.intensity_thresh + 
                                (1 - alpha) * new_params.intensity_thresh;
        params.diameter_min = alpha * params.diameter_min + 
                             (1 - alpha) * new_params.diameter_min;
        params.diameter_max = alpha * params.diameter_max + 
                             (1 - alpha) * new_params.diameter_max;
        params.max_points = static_cast<int>(
            alpha * params.max_points + (1 - alpha) * new_params.max_points);
        params.arc_threshold = alpha * params.arc_threshold + 
                             (1 - alpha) * new_params.arc_threshold;
    }
};

} // namespace amr_reflector_noise_handling