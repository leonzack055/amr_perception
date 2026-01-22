#pragma once

#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <numeric>

#include "types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief Result of geometric validation
 */
struct GeometricValidationResult {
    bool is_valid;
    double confidence;
    std::string rejection_reason;
    
    GeometricValidationResult()
        : is_valid(false)
        , confidence(0.0)
        , rejection_reason("")
    {}
};

/**
 * @brief Geometric validator for reflective posts
 * 
 * Validates that detected objects match the expected geometric properties
 * of a reflective post (circular, fixed diameter, uniform point distribution)
 */
class GeometricValidator {
public:
    /**
     * @brief Constructor with default parameters
     */
    GeometricValidator()
        : expected_diameter_(0.05)
        , diameter_tolerance_(0.03)
        , circularity_threshold_(0.5)
        , angle_diversity_threshold_(0.3)
    {}
    
    /**
     * @brief Constructor with custom parameters
     * @param expected_diameter Expected diameter of reflector (m)
     * @param diameter_tolerance Tolerance for diameter validation (m)
     * @param circularity_threshold Minimum circularity score
     * @param angle_diversity_threshold Minimum angle diversity score
     */
    GeometricValidator(double expected_diameter,
                    double diameter_tolerance,
                    double circularity_threshold = 0.5,
                    double angle_diversity_threshold = 0.3)
        : expected_diameter_(expected_diameter)
        , diameter_tolerance_(diameter_tolerance)
        , circularity_threshold_(circularity_threshold)
        , angle_diversity_threshold_(angle_diversity_threshold)
    {}
    
    /**
     * @brief Validate geometry of a detected cluster
     * @param points Points in the cluster
     * @param center Estimated center of the cluster
     * @return Validation result with confidence
     */
    GeometricValidationResult validateGeometry(const std::vector<Point>& points,
                                          const Point& center) {
        GeometricValidationResult result;
        
        if (points.size() < 3) {
            result.rejection_reason = "点数不足: " + std::to_string(points.size());
            return result;
        }
        
        // 1. 计算所有点到圆心的距离
        std::vector<double> distances;
        for (const auto& p : points) {
            double d = std::sqrt((p.x - center.x) * (p.x - center.x) + 
                                (p.y - center.y) * (p.y - center.y));
            distances.push_back(d);
        }
        
        // 2. 计算平均半径和标准差
        double mean_radius = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        double variance = 0.0;
        for (double d : distances) {
            variance += (d - mean_radius) * (d - mean_radius);
        }
        double std_radius = std::sqrt(variance / distances.size());
        
        // 3. 验证直径是否在合理范围内
        double diameter = 2 * mean_radius;
        if (!validateDiameter(diameter)) {
            result.rejection_reason = "直径超出范围: " + std::to_string(diameter) + "m (期望: " +
                                 std::to_string(expected_diameter_) + " ± " +
                                 std::to_string(diameter_tolerance_) + "m)";
            return result;
        }
        
        // 4. 验证圆形度（标准差应该较小）
        double circularity_score = computeCircularityScore(std_radius);
        if (circularity_score < circularity_threshold_) {
            result.rejection_reason = "圆形度不足: " + std::to_string(circularity_score) +
                                 " (阈值: " + std::to_string(circularity_threshold_) + ")";
            return result;
        }
        
        // 5. 验证角度分布（点应该均匀分布在圆周上）
        double angle_diversity = computeAngleDiversity(points, center);
        if (angle_diversity < angle_diversity_threshold_) {
            result.rejection_reason = "角度分布不均: " + std::to_string(angle_diversity) +
                                 " (阈值: " + std::to_string(angle_diversity_threshold_) + ")";
            return result;
        }
        
        // 6. 综合置信度
        result.confidence = computeOverallConfidence(circularity_score, angle_diversity);
        result.is_valid = result.confidence > 0.5;
        
        return result;
    }
    
    /**
     * @brief Set expected diameter
     */
    void setExpectedDiameter(double diameter) {
        expected_diameter_ = diameter;
    }
    
    /**
     * @brief Set diameter tolerance
     */
    void setDiameterTolerance(double tolerance) {
        diameter_tolerance_ = tolerance;
    }
    
    /**
     * @brief Set circularity threshold
     */
    void setCircularityThreshold(double threshold) {
        circularity_threshold_ = threshold;
    }
    
    /**
     * @brief Set angle diversity threshold
     */
    void setAngleDiversityThreshold(double threshold) {
        angle_diversity_threshold_ = threshold;
    }

private:
    double expected_diameter_;      // Expected diameter (m)
    double diameter_tolerance_;       // Tolerance for diameter (m)
    double circularity_threshold_;   // Minimum circularity score
    double angle_diversity_threshold_; // Minimum angle diversity score
    
    /**
     * @brief Validate diameter is within acceptable range
     */
    bool validateDiameter(double diameter) const {
        return diameter >= expected_diameter_ - diameter_tolerance_ &&
               diameter <= expected_diameter_ + diameter_tolerance_;
    }
    
    /**
     * @brief Compute circularity score based on radius standard deviation
     * @param std_radius Standard deviation of radii
     * @return Circularity score (0-1)
     */
    double computeCircularityScore(double std_radius) const {
        // 标准差2cm对应分数0.37，标准差越小分数越高
        return std::exp(-std_radius / 0.02);
    }
    
    /**
     * @brief Compute angle diversity score
     * @param points Points to analyze
     * @param center Center point
     * @return Angle diversity score (0-1)
     */
    double computeAngleDiversity(const std::vector<Point>& points,
                             const Point& center) const {
        if (points.size() < 3) return 0.5;
        
        std::vector<double> angles;
        for (const auto& p : points) {
            double dx = p.x - center.x;
            double dy = p.y - center.y;
            double angle = std::atan2(dy, dx);
            angles.push_back(angle);
        }
        
        std::sort(angles.begin(), angles.end());
        
        // 计算最大角度间隔
        double max_gap = 0;
        for (size_t i = 0; i < angles.size(); ++i) {
            double gap = angles[(i + 1) % angles.size()] - angles[i];
            if (gap < 0) gap += 2 * M_PI;
            max_gap = std::max(max_gap, gap);
        }
        
        // 最大间隔越小，分布越均匀，权重越高
        return 1.0 - (max_gap / (2 * M_PI));
    }
    
    /**
     * @brief Compute overall confidence from individual scores
     */
    double computeOverallConfidence(double circularity,
                                double angle_diversity) const {
        // 加权融合
        double fused = 0.5 * circularity + 0.5 * angle_diversity;
        
        // 应用非线性变换，提高高置信度的权重
        fused = std::pow(fused, 1.2);
        
        return std::clamp(fused, 0.0, 1.0);
    }
};

} // namespace amr_reflector_noise_handling