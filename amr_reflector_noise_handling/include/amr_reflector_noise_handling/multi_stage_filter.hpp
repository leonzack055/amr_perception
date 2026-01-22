#pragma once

#include <vector>
#include <string>
#include <memory>

#include "types.hpp"
#include "distance_adaptive_params.hpp"
#include "intensity_compensator.hpp"
#include "multi_scale_dbscan.hpp"
#include "geometric_validator.hpp"
#include <iostream>

namespace amr_reflector_noise_handling {

/**
 * @brief Result of multi-stage filtering
 */
struct FilterResult {
    std::vector<Point> filtered_points;  // Points after filtering
    double confidence;                  // Overall confidence (0-1)
    std::string stage_info;            // Information about filtering stages
    
    FilterResult()
        : confidence(0.0)
        , stage_info("")
    {}
};

/**
 * @brief Multi-stage filtering pipeline for reflective post detection
 * 
 * Implements a progressive filtering approach:
 * Stage 1: Intensity filtering
 * Stage 2: Point count range check
 * Stage 3: Arc feature filtering
 * Stage 4: Clustering
 * Stage 5: Geometric validation
 */
class MultiStageFilter {
public:
    /**
     * @brief Constructor
     */
    MultiStageFilter()
        : intensity_compensator_()
        , dbscan_()
        , geometric_validator_()
        , enable_stage1_(true)
        , enable_stage2_(true)
        , enable_stage3_(false)
        , enable_stage4_(true)
        , enable_stage5_(true)
    {}
    
    /**
     * @brief Process raw points through multi-stage filtering
     * @param raw_points Raw laser scan points
     * @param distance Estimated distance to reflector (m)
     * @return Filtered result with confidence
     */
    FilterResult process(const std::vector<Point>& raw_points, double distance) {
        FilterResult result;
        result.filtered_points = raw_points;
        result.confidence = 1.0;
        result.stage_info = "";
        
        // Get adaptive parameters
        auto params = computeAdaptiveParams(distance);
        
        // Stage 1: Intensity filtering
        if (enable_stage1_) {
            result.filtered_points = filterByIntensity(result.filtered_points, 
                                                   params.intensity_thresh);
            if (result.filtered_points.size() < static_cast<size_t>(params.min_points)) {
                result.confidence = 0.0;
                result.stage_info = "强度过滤: 点数不足 (" + 
                                   std::to_string(result.filtered_points.size()) + " < " +
                                   std::to_string(params.min_points) + ")";
                return result;
            }
            result.stage_info += "强度过滤(" + std::to_string(result.filtered_points.size()) + "点) -> ";
        }
        
        // Stage 2: Point count range check
        if (enable_stage2_) {
            if (result.filtered_points.size() > static_cast<size_t>(params.max_points)) {
                // Uniform sampling
                result.filtered_points = uniformSample(result.filtered_points, 
                                                  params.max_points);
            }
            result.stage_info += "点数检查(" + std::to_string(result.filtered_points.size()) + "点) -> ";
        }
        
        // Stage 3: Arc feature filtering (optional)
        if (enable_stage3_) {
            result.filtered_points = filterByArc(result.filtered_points, params.arc_threshold);
            result.stage_info += "弧度过滤 -> ";
        }
        
        // Stage 4: Clustering
        if (enable_stage4_) {
            auto clusters = dbscan_.cluster(result.filtered_points);
            
            if (clusters.empty()) {
                result.confidence = 0.0;
                result.stage_info += "聚类失败: 无有效聚类";
                return result;
            }
            
            // Select the largest cluster
            result.filtered_points.clear();
            for (int idx : clusters[0]) {
                result.filtered_points.push_back(raw_points[idx]);
            }
            result.stage_info += "聚类(" + std::to_string(result.filtered_points.size()) + "点) -> ";
        }
        
        // Stage 5: Geometric validation
        if (enable_stage5_) {
            Point center = computeCentroid(result.filtered_points);
            auto geo_result = geometric_validator_.validateGeometry(result.filtered_points, center);
            
            if (!geo_result.is_valid) {
                result.confidence = 0.0;
                result.stage_info += "几何验证失败: " + geo_result.rejection_reason;
                return result;
            }
            
            result.confidence = geo_result.confidence;
            result.stage_info += "几何验证通过(置信度:" + 
                               std::to_string(result.confidence) + ")";
        }
        
        return result;
    }
    
    /**
     * @brief Process multiple clusters from raw points
     * @param raw_points Raw laser scan points (with normalized intensities in [0,1])
     * @param distance Estimated distance to reflector (m)
     * @param normalized_intensity_threshold Intensity threshold for normalized values [0,1]
     * @return Vector of detected reflectors
     */
    std::vector<DetectedReflector> detectMultiple(
        const std::vector<Point>& raw_points,
        double distance,
        double normalized_intensity_threshold = 0.3) {
        
        std::vector<DetectedReflector> reflectors;
        
        // Get adaptive parameters
        auto params = computeAdaptiveParams(distance);
        
        // Stage 1: Intensity filtering using normalized intensity threshold
        auto filtered_points = filterByIntensity(raw_points, normalized_intensity_threshold);
        
        // Stage 2: Clustering
        auto clusters = dbscan_.cluster(filtered_points);
        
        // Stage 3: Validate each cluster
        for (const auto& cluster : clusters) {
            if (cluster.size() < static_cast<size_t>(params.min_points)) continue;
            
            std::vector<Point> cluster_points;
            for (int idx : cluster) {
                cluster_points.push_back(raw_points[idx]);
            }
            
            // Geometric validation
            Point center = computeCentroid(cluster_points);
            auto geo_result = geometric_validator_.validateGeometry(cluster_points, center);
            
            if (geo_result.is_valid) {
                DetectedReflector reflector;
                reflector.center = center;
                reflector.diameter = estimateDiameter(cluster_points, center);
                reflector.confidence = geo_result.confidence;
                reflector.point_count = cluster_points.size();
                reflector.mean_intensity = computeMeanIntensity(cluster_points);
                reflectors.push_back(reflector);
            } else {
                std::cerr << geo_result.rejection_reason << std::endl;
            }
        }
        
        return reflectors;
    }
    
    /**
     * @brief Enable/disable filtering stages
     */
    void setStage1Enabled(bool enable) { enable_stage1_ = enable; }
    void setStage2Enabled(bool enable) { enable_stage2_ = enable; }
    void setStage3Enabled(bool enable) { enable_stage3_ = enable; }
    void setStage4Enabled(bool enable) { enable_stage4_ = enable; }
    void setStage5Enabled(bool enable) { enable_stage5_ = enable; }
    
    /**
     * @brief Set geometric validator parameters
     */
    void setExpectedDiameter(double diameter) {
        geometric_validator_.setExpectedDiameter(diameter);
    }
    
    void setDiameterTolerance(double tolerance) {
        geometric_validator_.setDiameterTolerance(tolerance);
    }

private:
    IntensityCompensator intensity_compensator_;
    MultiScaleDBSCAN dbscan_;
    GeometricValidator geometric_validator_;
    
    bool enable_stage1_;  // Intensity filtering
    bool enable_stage2_;  // Point count check
    bool enable_stage3_;  // Arc filtering
    bool enable_stage4_;  // Clustering
    bool enable_stage5_;  // Geometric validation
    
    /**
     * @brief Filter points by intensity threshold
     */
    std::vector<Point> filterByIntensity(const std::vector<Point>& points,
                                      double threshold) const {
        std::vector<Point> filtered;
        for (const auto& p : points) {
            if (p.intensity >= threshold) {
                filtered.push_back(p);
            }
        }
        return filtered;
    }
    
    /**
     * @brief Uniform sampling of points
     */
    std::vector<Point> uniformSample(const std::vector<Point>& points,
                                    size_t max_count) const {
        if (points.size() <= max_count) return points;
        
        std::vector<Point> sampled;
        double step = static_cast<double>(points.size()) / max_count;
        
        for (size_t i = 0; i < max_count; ++i) {
            size_t index = static_cast<size_t>(i * step);
            if (index < points.size()) {
                sampled.push_back(points[index]);
            }
        }
        
        return sampled;
    }
    
    /**
     * @brief Filter by arc feature (placeholder for future implementation)
     */
    std::vector<Point> filterByArc(const std::vector<Point>& points,
                                   double threshold) const {
        // TODO: Implement arc-based filtering
        return points;
    }
    
    /**
     * @brief Compute centroid of points
     */
    Point computeCentroid(const std::vector<Point>& points) const {
        if (points.empty()) return Point();
        
        double sum_x = 0, sum_y = 0;
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        
        return Point(sum_x / points.size(), sum_y / points.size());
    }
    
    /**
     * @brief Estimate diameter from points
     */
    double estimateDiameter(const std::vector<Point>& points,
                         const Point& center) const {
        if (points.empty()) return 0.0;
        
        double max_distance = 0.0;
        for (const auto& p : points) {
            double dx = p.x - center.x;
            double dy = p.y - center.y;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d > max_distance) {
                max_distance = d;
            }
        }
        
        return 2.0 * max_distance;
    }
    
    /**
     * @brief Compute mean intensity
     */
    double computeMeanIntensity(const std::vector<Point>& points) const {
        if (points.empty()) return 0.0;
        
        double sum = 0.0;
        for (const auto& p : points) {
            sum += p.intensity;
        }
        
        return sum / points.size();
    }
};

} // namespace amr_reflector_noise_handling