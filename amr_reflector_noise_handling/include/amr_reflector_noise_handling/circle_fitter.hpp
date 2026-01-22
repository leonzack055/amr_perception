#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include "amr_reflector_noise_handling/types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief 圆拟合结果
 */
struct CircleFitResult {
    Point center;
    double radius;
    double fit_error;           // 拟合误差（RMS）
    int inlier_count;           // 内点数量
    int total_points;           // 总点数
    double inlier_ratio;        // 内点比例
    bool is_valid;              // 是否有效
};

/**
 * @brief 圆拟合参数
 */
struct CircleFitParams {
    double max_fit_error = 0.03;          // 最大拟合误差3cm
    double min_inlier_ratio = 0.5;        // 最小内点比例50%
    double max_fit_error_near = 0.04;      // 近距离最大误差4cm
    double max_fit_error_far = 0.02;       // 远距离最大误差2cm
    double far_distance_threshold = 3.0;   // 远距离阈值3m
    double min_radius = 0.02;             // 最小半径5cm
    double max_radius = 0.09;             // 最大半径9cm
    double inlier_threshold = 0.02;       // 内点阈值2cm
};

/**
 * @brief 圆拟合器
 * 
 * 对插值后的点云进行圆拟合，用于验证反光柱候选
 */
class CircleFitter {
public:
    CircleFitter() = default;
    
    /**
     * @brief 设置拟合参数
     */
    void setParams(const CircleFitParams& params) {
        params_ = params;
    }
    
    /**
     * @brief 简单迭代圆拟合
     * 
     * @param points 点云
     * @return CircleFitResult 拟合结果
     */
    CircleFitResult fitCircle(const std::vector<Point>& points) const {
        CircleFitResult result;
        result.total_points = points.size();
        result.is_valid = false;
        
        if (points.size() < 3) {
            result.fit_error = 999;
            return result;
        }
        
        // 1. 计算初始中心（简单平均）
        double sum_x = 0, sum_y = 0;
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        result.center.x = sum_x / points.size();
        result.center.y = sum_y / points.size();
        
        // 2. 迭代优化（5次迭代足够）
        for (int iter = 0; iter < 5; ++iter) {
            // 2.1 计算每个点到中心的距离
            std::vector<double> distances;
            for (const auto& p : points) {
                distances.push_back(p.distanceTo(result.center));
            }
            
            // 2.2 计算加权中心（距离越近权重越大）
            double sum_wx = 0, sum_wy = 0, sum_w = 0;
            
            for (size_t i = 0; i < points.size(); ++i) {
                double weight = 1.0 / (distances[i] + 1e-6);
                sum_wx += weight * points[i].x;
                sum_wy += weight * points[i].y;
                sum_w += weight;
            }
            
            if (sum_w > 1e-6) {
                result.center.x = sum_wx / sum_w;
                result.center.y = sum_wy / sum_w;
            }
        }
        
        // 3. 计算半径（所有点到中心的平均距离）
        std::vector<double> all_distances;
        for (const auto& p : points) {
            double d = p.distanceTo(result.center);
            all_distances.push_back(d);
        }
        
        result.radius = std::accumulate(all_distances.begin(), all_distances.end(), 0.0) / all_distances.size();
        
        // 4. 计算拟合误差和内点
        double error_sum = 0;
        result.inlier_count = 0;
        for (double d : all_distances) {
            double error = std::abs(d - result.radius);
            error_sum += error * error;
            
            if (std::abs(d - result.radius) < params_.inlier_threshold) {
                result.inlier_count++;
            }
        }
        result.fit_error = std::sqrt(error_sum / points.size());
        result.inlier_ratio = static_cast<double>(result.inlier_count) / points.size();
        
        return result;
    }
    
    /**
     * @brief 验证圆拟合结果
     * 
     * @param fit 拟合结果
     * @param original_point_count 原始点数
     * @param distance 距离
     * @return bool 是否有效
     */
    bool validateFit(
        const CircleFitResult& fit,
        int original_point_count,
        double distance) const {
        
        // 1. 拟合误差检查
        double max_error = params_.max_fit_error;
        
        // 根据距离调整误差阈值
        if (distance < params_.far_distance_threshold) {
            max_error = params_.max_fit_error_near;  // 近距离允许更大误差
        } else {
            max_error = params_.max_fit_error_far;   // 远距离要求更小误差
        }
        
        if (fit.fit_error > max_error) {
            return false;
        }
        
        // 2. 半径合理性（7cm ± 2cm）
        if (fit.radius < params_.min_radius || fit.radius > params_.max_radius) {
            return false;
        }
        
        // 3. 内点比例（考虑插值点）
        // 插值后点数约1.5倍原始点数
        double expected_points = original_point_count * 1.5;
        double expected_inlier_ratio = params_.min_inlier_ratio;
        
        double actual_inlier_ratio = static_cast<double>(fit.inlier_count) / expected_points;
        if (actual_inlier_ratio < expected_inlier_ratio) {
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 计算圆的置信度
     * 
     * @param fit 拟合结果
     * @param angular_coverage 角度覆盖率
     * @return double 置信度[0, 1]
     */
    double computeConfidence(
        const CircleFitResult& fit,
        double angular_coverage) const {
        
        // 1. 拟合误差贡献（权重0.4）
        double error_score = std::exp(-fit.fit_error / 0.02);
        
        // 2. 内点比例贡献（权重0.3）
        double inlier_score = fit.inlier_ratio;
        
        // 3. 角度覆盖贡献（权重0.3）
        double coverage_score = angular_coverage / (2 * M_PI);
        
        // 4. 综合置信度
        double confidence = 0.4 * error_score + 0.3 * inlier_score + 0.3 * coverage_score;
        
        return std::clamp(confidence, 0.0, 1.0);
    }
    
    /**
     * @brief 计算点到圆的残差
     * 
     * @param point 点
     * @param fit 拟合结果
     * @return double 残差
     */
    double computeResidual(const Point& point, const CircleFitResult& fit) const {
        double dist = point.distanceTo(fit.center);
        return std::abs(dist - fit.radius);
    }
    
private:
    CircleFitParams params_;
};

} // namespace amr_reflector_noise_handling