#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include "amr_reflector_noise_handling/types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief 实用反光柱描述子
 * 
 * 基于圆拟合结果和分形维数的描述子，用于反光柱跟踪
 */
struct PracticalReflectorDescriptor {
    // 圆形拟合结果
    Point circle_center;
    double circle_radius;
    double fit_error;
    double inlier_ratio;
    
    // 分形维数
    double fractal_dimension;
    double cv;  // 变异系数
    
    // 点云特征
    int point_count_original;      // 原始点数
    int point_count_interpolated;  // 插值后点数
    double mean_intensity;
    double intensity_std;
    
    // 几何特征
    double density;               // 点云密度
    double angular_coverage;       // 角度覆盖率
    
    // 元数据
    double timestamp;
    
    /**
     * @brief 计算与另一个描述子的相似度
     * 
     * @param other 另一个描述子
     * @return double 相似度[0, 1]
     */
    double similarity(const PracticalReflectorDescriptor& other) const {
        double score = 0.0;
        
        // 1. 半径相似度（权重0.4）
        double radius_diff = std::abs(circle_radius - other.circle_radius);
        score += 0.4 * std::exp(-radius_diff / 0.015);
        
        // 2. 强度相似度（权重0.25）
        double intensity_diff = std::abs(mean_intensity - other.mean_intensity);
        score += 0.25 * std::exp(-intensity_diff / 50.0);
        
        // 3. 分形维数相似度（权重0.2）
        double fd_diff = std::abs(fractal_dimension - other.fractal_dimension);
        score += 0.2 * std::exp(-fd_diff / 0.3);
        
        // 4. 密度相似度（权重0.1）
        double density_diff = std::abs(density - other.density);
        score += 0.1 * std::exp(-density_diff / 10.0);
        
        // 5. 角度覆盖相似度（权重0.05）
        double coverage_diff = std::abs(angular_coverage - other.angular_coverage);
        score += 0.05 * std::exp(-coverage_diff / 0.5);
        
        return score;
    }
    
    /**
     * @brief 指数平滑更新描述子
     * 
     * @param new_desc 新描述子
     * @param alpha 平滑系数（0-1）
     */
    void update(const PracticalReflectorDescriptor& new_desc, double alpha = 0.3) {
        circle_center.x = (1 - alpha) * circle_center.x + alpha * new_desc.circle_center.x;
        circle_center.y = (1 - alpha) * circle_center.y + alpha * new_desc.circle_center.y;
        circle_radius = (1 - alpha) * circle_radius + alpha * new_desc.circle_radius;
        fit_error = (1 - alpha) * fit_error + alpha * new_desc.fit_error;
        inlier_ratio = (1 - alpha) * inlier_ratio + alpha * new_desc.inlier_ratio;
        fractal_dimension = (1 - alpha) * fractal_dimension + alpha * new_desc.fractal_dimension;
        cv = (1 - alpha) * cv + alpha * new_desc.cv;
        mean_intensity = (1 - alpha) * mean_intensity + alpha * new_desc.mean_intensity;
        intensity_std = (1 - alpha) * intensity_std + alpha * new_desc.intensity_std;
        density = (1 - alpha) * density + alpha * new_desc.density;
        angular_coverage = (1 - alpha) * angular_coverage + alpha * new_desc.angular_coverage;
        timestamp = new_desc.timestamp;
    }
};

/**
 * @brief 实用描述子提取器
 */
class PracticalDescriptorExtractor {
public:
    PracticalDescriptorExtractor() = default;
    
    /**
     * @brief 从聚类和拟合结果提取描述子
     * 
     * @param original_cluster 原始聚类
     * @param interpolated_cluster 插值后聚类
     * @param circle_fit 圆拟合结果
     * @param fd_result 分形维数结果
     * @param angular_coverage 角度覆盖率
     * @param timestamp 时间戳
     * @return PracticalReflectorDescriptor 描述子
     */
    PracticalReflectorDescriptor extract(
        const std::vector<Point>& original_cluster,
        const std::vector<Point>& interpolated_cluster,
        const struct CircleFitResult& circle_fit,
        const struct FractalDimensionResult& fd_result,
        double angular_coverage,
        double timestamp) const {
        
        PracticalReflectorDescriptor desc;
        
        // 1. 圆形拟合结果
        desc.circle_center = circle_fit.center;
        desc.circle_radius = circle_fit.radius;
        desc.fit_error = circle_fit.fit_error;
        desc.inlier_ratio = circle_fit.inlier_ratio;
        
        // 2. 分形维数
        desc.fractal_dimension = fd_result.dimension;
        desc.cv = fd_result.cv;
        
        // 3. 点云特征
        desc.point_count_original = original_cluster.size();
        desc.point_count_interpolated = interpolated_cluster.size();
        desc.mean_intensity = computeMeanIntensity(original_cluster);
        desc.intensity_std = computeIntensityStd(original_cluster);
        
        // 4. 几何特征
        desc.density = original_cluster.size() / (circle_fit.radius * circle_fit.radius + 1e-6);
        desc.angular_coverage = angular_coverage;
        
        // 5. 时间戳
        desc.timestamp = timestamp;
        
        return desc;
    }
    
    /**
     * @brief 计算描述子的置信度
     * 
     * @param desc 描述子
     * @param expected_fd 期望分形维数
     * @return double 置信度[0, 1]
     */
    double computeConfidence(
        const PracticalReflectorDescriptor& desc,
        double expected_fd = 1.15) const {
        
        double confidence = 0.0;
        
        // 1. 拟合误差（权重0.3）
        double error_score = std::exp(-desc.fit_error / 0.02);
        confidence += 0.3 * error_score;
        
        // 2. 内点比例（权重0.25）
        confidence += 0.25 * desc.inlier_ratio;
        
        // 3. 角度覆盖（权重0.2）
        double coverage_score = desc.angular_coverage / (2 * M_PI);
        confidence += 0.2 * coverage_score;
        
        // 4. 分形维数（权重0.15）
        double fd_diff = std::abs(desc.fractal_dimension - expected_fd);
        double fd_score = std::exp(-fd_diff / 0.2);
        confidence += 0.15 * fd_score;
        
        // 5. 半径合理性（权重0.1）
        // 期望半径3.5cm（7cm直径）
        double radius_diff = std::abs(desc.circle_radius - 0.035);
        double radius_score = std::exp(-radius_diff / 0.015);
        confidence += 0.1 * radius_score;
        
        return std::clamp(confidence, 0.0, 1.0);
    }
    
private:
    /**
     * @brief 计算平均强度
     */
    double computeMeanIntensity(const std::vector<Point>& points) const {
        if (points.empty()) {
            return 0.0;
        }
        
        double sum = 0;
        for (const auto& p : points) {
            sum += p.intensity;
        }
        return sum / points.size();
    }
    
    /**
     * @brief 计算强度标准差
     */
    double computeIntensityStd(const std::vector<Point>& points) const {
        if (points.size() < 2) {
            return 0.0;
        }
        
        double mean = computeMeanIntensity(points);
        double sum_sq = 0;
        for (const auto& p : points) {
            sum_sq += (p.intensity - mean) * (p.intensity - mean);
        }
        return std::sqrt(sum_sq / points.size());
    }
};

} // namespace amr_reflector_noise_handling