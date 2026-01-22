#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "amr_reflector_noise_handling/types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief 改进的插值补偿器
 * 
 * 基于角分辨率和圆弧的智能插值，适用于稀疏点云
 */
class ImprovedInterpolationCompensator {
public:
    struct InterpolationParams {
        size_t min_points = 5;       // 最小点数
        size_t max_points = 20;      // 最大点数
        double min_distance = 2.0;   // 最小插值距离2m
        double max_distance = 4.0;   // 最大插值距离4m
        double gap_multiplier = 1.5;  // 间隙倍数
    };
    
    ImprovedInterpolationCompensator() = default;
    
    /**
     * @brief 设置插值参数
     */
    void setParams(const InterpolationParams& params) {
        params_ = params;
    }
    
    /**
     * @brief 判断是否需要插值
     * 
     * @param cluster 原始聚类
     * @param center 聚类中心
     * @param distance 距离
     * @return bool 是否需要插值
     */
    bool shouldInterpolate(
        const std::vector<Point>& cluster,
        const Point& center,
        double distance) const {
        
        // 1. 点数条件：5-20个点
        if (cluster.size() < params_.min_points || cluster.size() > params_.max_points) {
            return false;
        }
        
        // 2. 距离条件：2-4m范围才插值
        if (distance < params_.min_distance || distance > params_.max_distance) {
            return false;
        }
        
        // 3. 计算角分辨率
        auto angles = computeAngles(cluster, center);
        if (angles.size() < 2) {
            return false;
        }
        
        // 4. 计算平均角度间隙
        std::vector<double> gaps = computeAngleGaps(angles);
        double mean_gap = std::accumulate(gaps.begin(), gaps.end(), 0.0) / gaps.size();
        
        // 5. 判断是否需要插值
        // 期望间隙：2π / (点数 * 1.2)
        double expected_gap = 2 * M_PI / (cluster.size() * 1.2);
        
        // 如果实际间隙大于期望值的倍数，说明点太稀疏，需要插值
        return (mean_gap > expected_gap * params_.gap_multiplier);
    }
    
    /**
     * @brief 对聚类进行插值补偿
     * 
     * @param cluster 原始聚类
     * @param center 聚类中心
     * @param distance 距离
     * @return std::vector<Point> 插值后的点云
     */
    std::vector<Point> interpolateCluster(
        const std::vector<Point>& cluster,
        const Point& center,
        double distance) const {
        
        if (!shouldInterpolate(cluster, center, distance)) {
            return cluster;  // 不需要插值
        }
        
        // 1. 按角度排序
        auto sorted_indices = sortIndicesByAngle(cluster, center);
        
        std::vector<Point> interpolated;
        
        // 2. 在每对相邻点之间检查是否需要插值
        for (size_t i = 0; i < sorted_indices.size(); ++i) {
            size_t idx = sorted_indices[i];
            size_t next_idx = sorted_indices[(i + 1) % sorted_indices.size()];
            
            const Point& p1 = cluster[idx];
            const Point& p2 = cluster[next_idx];
            
            double angle1 = std::atan2(p1.y - center.y, p1.x - center.x);
            double angle2 = std::atan2(p2.y - center.y, p2.x - center.x);
            double gap = angle2 - angle1;
            if (gap < 0) gap += 2 * M_PI;
            
            // 添加原始点
            interpolated.push_back(p1);
            
            // 3. 期望间隙：2π / (点数 * 1.2)
            double expected_gap = 2 * M_PI / (cluster.size() * 1.2);
            
            // 4. 如果间隙过大，插值
            if (gap > expected_gap * params_.gap_multiplier) {
                int num_interp = static_cast<int>(gap / expected_gap);
                
                // 限制插值数量
                num_interp = std::min(num_interp, 10);
                
                for (int j = 1; j < std::max(1, num_interp); ++j) {
                    double interp_angle = angle1 + gap * j / num_interp;
                    
                    // 插值点在以center为圆心的圆上
                    // 半径取两个原始点距离的平均
                    double r1 = p1.distanceTo(center);
                    double r2 = p2.distanceTo(center);
                    double r = (r1 + r2) / 2.0;
                    
                    Point interp_point;
                    interp_point.x = center.x + r * std::cos(interp_angle);
                    interp_point.y = center.y + r * std::sin(interp_angle);
                    interp_point.intensity = (p1.intensity + p2.intensity) / 2.0;
                    interpolated.push_back(interp_point);
                }
            }
        }
        
        return interpolated;
    }
    
    /**
     * @brief 计算点云的平均角分辨率
     * 
     * @param cluster 点云
     * @param center 中心
     * @return double 平均角分辨率（弧度）
     */
    double computeAngularResolution(
        const std::vector<Point>& cluster,
        const Point& center) const {
        
        auto angles = computeAngles(cluster, center);
        if (angles.size() < 2) {
            return 0.0;
        }
        
        auto gaps = computeAngleGaps(angles);
        return std::accumulate(gaps.begin(), gaps.end(), 0.0) / gaps.size();
    }
    
    /**
     * @brief 估计期望点数（基于距离）
     * 
     * @param distance 距离
     * @return int 期望点数
     */
    int estimateExpectedPoints(double distance) const {
        // 2m范围：期望10-20点
        // 4m范围：期望5-10点
        // 线性插值
        double ratio = (distance - params_.min_distance) / 
                      (params_.max_distance - params_.min_distance);
        ratio = std::clamp(ratio, 0.0, 1.0);
        
        int max_expected = 20;
        int min_expected = 5;
        
        return static_cast<int>(max_expected - ratio * (max_expected - min_expected));
    }
    
private:
    InterpolationParams params_;
    
    /**
     * @brief 计算所有点的角度
     */
    std::vector<double> computeAngles(
        const std::vector<Point>& cluster,
        const Point& center) const {
        
        std::vector<double> angles;
        for (const auto& p : cluster) {
            angles.push_back(std::atan2(p.y - center.y, p.x - center.x));
        }
        std::sort(angles.begin(), angles.end());
        return angles;
    }
    
    /**
     * @brief 计算角度间隙
     */
    std::vector<double> computeAngleGaps(const std::vector<double>& angles) const {
        std::vector<double> gaps;
        for (size_t i = 0; i < angles.size(); ++i) {
            double gap = angles[(i + 1) % angles.size()] - angles[i];
            if (gap < 0) gap += 2 * M_PI;
            gaps.push_back(gap);
        }
        return gaps;
    }
    
    /**
     * @brief 按角度排序点云（返回排序后的索引）
     */
    std::vector<size_t> sortIndicesByAngle(
        const std::vector<Point>& cluster,
        const Point& center) const {
        
        std::vector<size_t> indices(cluster.size());
        std::iota(indices.begin(), indices.end(), 0);
        
        std::sort(indices.begin(), indices.end(),
            [&cluster, &center](size_t i, size_t j) {
                double angle_i = std::atan2(cluster[i].y - center.y, cluster[i].x - center.x);
                double angle_j = std::atan2(cluster[j].y - center.y, cluster[j].x - center.x);
                return angle_i < angle_j;
            });
        
        return indices;
    }
};

} // namespace amr_reflector_noise_handling
