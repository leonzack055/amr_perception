#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include "amr_reflector_noise_handling/types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief 分形维数计算结果
 */
struct FractalDimensionResult {
    double dimension;       // 分形维数 [1.0, 2.0]
    double linear_fit_r2;   // 线性拟合R²
    bool is_reliable;       // 是否可靠（点数足够）
    double mean_distance;   // 平均距离
    double std_distance;    // 距离标准差
    double cv;              // 变异系数
};

/**
 * @brief 聚类类型分类
 */
enum ClusterType {
    REFLECTOR_POST_CANDIDATE,  // 反光柱候选
    REFLECTOR_BOARD_CANDIDATE,  // 反光板候选
    NOISE_CLUSTER,               // 噪声簇
    UNKNOWN
};

/**
 * @brief 分类参数
 */
struct ClassificationParams {
    double min_fd_for_post = 1.0;    // 最小分形维数（反光柱）
    double max_fd_for_post = 1.3;    // 最大分形维数（反光柱）
    double min_density = 50.0;       // 最小点云密度
};

/**
 * @brief 分形维数计算器
 *
 * 用于基于点云的分布特征进行聚类分类，区分反光柱、反光板和噪声
 */
class FractalDimensionCalculator {
public:
    FractalDimensionCalculator() = default;
    
    /**
     * @brief 计算简化版分形维数（基于距离分布）
     * 
     * @param points 点云
     * @param center 聚类中心
     * @return FractalDimensionResult 分形维数结果
     */
    FractalDimensionResult computeSimplified(
        const std::vector<Point>& points,
        const Point& center) const {
        
        FractalDimensionResult result;
        
        if (points.size() < 3) {
            result.dimension = 2.0;
            result.is_reliable = false;
            return result;
        }
        
        // 1. 计算所有点到中心的距离
        std::vector<double> distances;
        for (const auto& p : points) {
            distances.push_back(p.distanceTo(center));
        }
        
        // 2. 计算距离的统计特性
        double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        result.mean_distance = mean;
        
        double variance = 0;
        for (double d : distances) {
            variance += (d - mean) * (d - mean);
        }
        variance /= distances.size();
        result.std_distance = std::sqrt(variance);
        
        // 3. 计算变异系数（Coefficient of Variation）
        result.cv = (mean > 1e-6) ? (result.std_distance / mean) : 0.0;
        
        // 4. 计算分形维数
        // 反光柱：距离集中，cv小 → 维数接近1
        // 噪声：距离分散，cv大 → 维数接近2
        double dimension = 1.0 + result.cv * 1.5;
        
        // 5. 归一化到[1, 2]
        result.dimension = std::clamp(dimension, 1.0, 2.0);
        
        // 6. 可靠性判断
        result.linear_fit_r2 = 0.9;  // 简化版默认高拟合质量
        result.is_reliable = (points.size() >= 5);
        
        return result;
    }
    
    /**
     * @brief 计算点云密度
     * 
     * @param points 点云
     * @param center 聚类中心
     * @return double 密度（点数/面积）
     */
    double computeDensity(
        const std::vector<Point>& points,
        const Point& center) const {
        
        if (points.empty()) {
            return 0.0;
        }
        
        // 计算最大距离
        double max_dist = 0;
        for (const auto& p : points) {
            double dist = p.distanceTo(center);
            max_dist = std::max(max_dist, dist);
        }
        
        if (max_dist < 1e-6) {
            return 0.0;
        }
        
        // 密度 = 点数 / (π * r²)
        double area = M_PI * max_dist * max_dist;
        return points.size() / area;
    }
    
    /**
     * @brief 计算角度覆盖率和最大间隙
     * 
     * @param points 点云
     * @param center 聚类中心
     * @return std::pair<double, double> {角度覆盖率, 最大角度间隙}
     */
    std::pair<double, double> computeAngularCoverage(
        const std::vector<Point>& points,
        const Point& center) const {
        
        if (points.size() < 2) {
            return {0.0, 2 * M_PI};
        }
        
        // 1. 计算所有角度
        std::vector<double> angles;
        for (const auto& p : points) {
            angles.push_back(std::atan2(p.y - center.y, p.x - center.x));
        }
        std::sort(angles.begin(), angles.end());
        
        // 2. 计算最大角度间隙
        double max_gap = 0;
        for (size_t i = 0; i < angles.size(); ++i) {
            double gap = angles[(i + 1) % angles.size()] - angles[i];
            if (gap < 0) gap += 2 * M_PI;
            max_gap = std::max(max_gap, gap);
        }
        
        // 3. 计算角度覆盖率
        double coverage = 2 * M_PI - max_gap;
        
        return {coverage, max_gap};
    }
    
    /**
     * @brief 基于分形维数分类聚类
     * 
     * @param points 点云
     * @param center 聚类中心
     * @param params 分类参数
     * @return ClusterType 聚类类型
     */
    ClusterType classifyCluster(
        const std::vector<Point>& points,
        const Point& center,
        const struct ClassificationParams& params) const {
        
        // 1. 计算分形维数
        auto fd_result = computeSimplified(points, center);
        
        // 2. 计算点云密度
        double density = computeDensity(points, center);
        
        // 3. 计算角度分布
        auto [coverage, max_gap] = computeAngularCoverage(points, center);
        
        // 4. 分类规则
        
        // 规则1: 点数过少 → 噪声
        if (points.size() < 3) {
            return NOISE_CLUSTER;
        }
        
        // 规则2: 分形维数[1.0, 1.3] + 密度适中 → 反光柱候选
        if (fd_result.dimension >= params.min_fd_for_post &&
            fd_result.dimension <= params.max_fd_for_post &&
            density >= params.min_density) {
            return REFLECTOR_POST_CANDIDATE;
        }
        
        // 规则3: 分形维数<1.0 + 角度覆盖大 → 反光板
        if (fd_result.dimension < params.min_fd_for_post && max_gap < M_PI) {
            return REFLECTOR_BOARD_CANDIDATE;
        }
        
        // 规则4: 分形维数>1.5 + 点数少 → 噪声
        if (fd_result.dimension > 1.5 && points.size() < 8) {
            return NOISE_CLUSTER;
        }
        
        // 规则5: 密度过低 → 噪声
        if (density < params.min_density * 0.3) {
            return NOISE_CLUSTER;
        }
        
        return UNKNOWN;
    }
    
private:
};

} // namespace amr_reflector_noise_handling