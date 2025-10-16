#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <functional>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <numeric>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <queue>
#include <unordered_set>
#include <map>
#include "common/reflector_common.hpp"
namespace landmark_localization
{

// 圆心结构体
struct CircleCenter {
    double x;
    double y;
    double timestamp;
    double t;
    double r;
    double c;
    double d;
    int id; // 通过聚类分配的临时ID
    
    CircleCenter(double x_, double y_, double ts_,double t_, double r_, double c_, double d_, int id_ = -1) 
        : x(x_), y(y_), timestamp(ts_), t(t_),r(r_),c(c_),d(d_),id(id_) {}
};

// 相对坐标系下的滑窗平滑器
class RelativeCircleSmoother {
private:
    std::deque<std::vector<CircleCenter>> window_;
    size_t window_size_;
    double max_time_gap_;
    int next_id_ = 0;
    
public:
    RelativeCircleSmoother(size_t window_size = 5, double max_time_gap = 0.5) 
        : window_size_(window_size), max_time_gap_(max_time_gap) {}
    
    // 处理新的一帧数据
    std::vector<CircleCenter> processFrame(const std::vector<CircleCenter>& new_frame) {
        // 为当前帧分配临时ID
        std::vector<CircleCenter> frame_with_id = assignTemporaryIDs(new_frame);
        
        // 添加到滑窗
        window_.push_back(frame_with_id);
        if (window_.size() > window_size_) {
            window_.pop_front();
        }
        
        // 如果窗口太小，直接返回
        if (window_.size() < 2) {
            return frame_with_id;
        }
        
        return smoothRelativeCircles();
    }
    
    void clear() {
        window_.clear();
        next_id_ = 0;
    }

private:
    // 为当前帧分配临时ID（基于相对位置聚类）
    std::vector<CircleCenter> assignTemporaryIDs(const std::vector<CircleCenter>& frame) {
        if (window_.empty()) {
            // 第一帧，直接分配新ID
            std::vector<CircleCenter> result;
            for (const auto& center : frame) {
                result.emplace_back(center.x, center.y, center.timestamp, center.t,center.r,center.c,center.d,next_id_++);
            }
            return result;
        }
        
        // 基于相对位置匹配上一帧的ID
        const auto& last_frame = window_.back();
        std::vector<CircleCenter> result;
        std::vector<bool> matched(last_frame.size(), false);
        
        for (const auto& new_center : frame) {
            int best_match_id = -1;
            double min_distance = std::numeric_limits<double>::max();
            
            // 在上一帧中寻找最近的点
            for (size_t i = 0; i < last_frame.size(); ++i) {
                if (matched[i]) continue;
                
                double dist = calculateDistance(new_center, last_frame[i]);
                if (dist < 0.3 && dist < min_distance) { // 30cm匹配阈值
                    min_distance = dist;
                    best_match_id = last_frame[i].id;
                }
            }
            
            if (best_match_id != -1) {
                // 找到匹配，继承ID
                result.emplace_back(new_center.x, new_center.y, new_center.timestamp, new_center.t,new_center.r,new_center.c,new_center.d,best_match_id);
                // 标记为已匹配
                for (size_t i = 0; i < last_frame.size(); ++i) {
                    if (last_frame[i].id == best_match_id) {
                        matched[i] = true;
                        break;
                    }
                }
            } else {
                // 新目标，分配新ID
                result.emplace_back(new_center.x, new_center.y, new_center.timestamp, new_center.t,new_center.r,new_center.c,new_center.d,next_id_++);
            }
        }
        
        return result;
    }
    
    // 相对坐标系下的平滑算法
    std::vector<CircleCenter> smoothRelativeCircles() {
        const auto& latest_frame = window_.back();
        std::vector<CircleCenter> smoothed_centers;
        
        for (const auto& latest_center : latest_frame) {
            // 收集该ID的历史轨迹
            auto trajectory = collectTrajectoryByID(latest_center.id);
            
            if (trajectory.size() >= 2) {
                // 使用多种策略进行平滑
                CircleCenter smoothed = adaptiveSmoothing(trajectory);
                smoothed_centers.push_back(smoothed);
            } else {
                smoothed_centers.push_back(latest_center);
            }
        }
        
        return smoothed_centers;
    }
    
    // 按ID收集历史轨迹
    std::vector<CircleCenter> collectTrajectoryByID(int id) {
        std::vector<CircleCenter> trajectory;
        
        for (auto it = window_.rbegin(); it != window_.rend(); ++it) {
            for (const auto& center : *it) {
                if (center.id == id) {
                    trajectory.push_back(center);
                    break;
                }
            }
        }
        
        // 按时间排序（从旧到新）
        std::reverse(trajectory.begin(), trajectory.end());
        return trajectory;
    }
    
    // 自适应平滑策略
    CircleCenter adaptiveSmoothing(const std::vector<CircleCenter>& trajectory) {
        if (trajectory.size() == 2) {
            // 只有两帧时使用简单平均
            return simpleAverage(trajectory);
        }
        
        // 计算移动速度
        double speed = calculateMovementSpeed(trajectory);
        
        if (speed < 0.1) { // 低速移动，使用强平滑
            return lowPassFilter(trajectory, 0.8);
        } else if (speed < 0.5) { // 中速移动，使用加权平均
            return weightedAverage(trajectory);
        } else { // 高速移动，使用弱平滑
            return lowPassFilter(trajectory, 0.3);
        }
    }
    
    // 计算移动速度
    double calculateMovementSpeed(const std::vector<CircleCenter>& trajectory) {
        double total_distance = 0;
        double total_time = 0;
        
        for (size_t i = 1; i < trajectory.size(); ++i) {
            double dist = calculateDistance(trajectory[i], trajectory[i-1]);
            double time_diff = trajectory[i].timestamp - trajectory[i-1].timestamp;
            
            total_distance += dist;
            total_time += time_diff;
        }
        
        if (total_time < 1e-6) return 0;
        return total_distance / total_time;
    }
    
    // 简单平均
    CircleCenter simpleAverage(const std::vector<CircleCenter>& trajectory) {
        double sum_x = 0, sum_y = 0;
        for (const auto& center : trajectory) {
            sum_x += center.x;
            sum_y += center.y;
        }
        
        const auto& latest = trajectory.back();
        return CircleCenter(sum_x / trajectory.size(), sum_y / trajectory.size(), 
                          latest.timestamp, latest.t,latest.r,latest.c,latest.d,latest.id);
    }
    
    // 低通滤波器
    CircleCenter lowPassFilter(const std::vector<CircleCenter>& trajectory, double alpha) {
        // 使用指数平滑
        double x = trajectory[0].x;
        double y = trajectory[0].y;
        
        for (size_t i = 1; i < trajectory.size(); ++i) {
            x = alpha * trajectory[i].x + (1 - alpha) * x;
            y = alpha * trajectory[i].y + (1 - alpha) * y;
        }
        
        const auto& latest = trajectory.back();
        return CircleCenter(x, y, latest.timestamp, latest.t,latest.r,latest.c,latest.d,latest.id);
    }
    
    // 加权平均（时间越近权重越大）
    CircleCenter weightedAverage(const std::vector<CircleCenter>& trajectory) {
        double total_weight = 0;
        double sum_x = 0, sum_y = 0;
        double latest_time = trajectory.back().timestamp;
        
        for (size_t i = 0; i < trajectory.size(); ++i) {
            // 指数衰减权重
            double time_diff = latest_time - trajectory[i].timestamp;
            double weight = std::exp(-time_diff * 2.0); // 衰减因子可调
            
            sum_x += weight * trajectory[i].x;
            sum_y += weight * trajectory[i].y;
            total_weight += weight;
        }
        
        const auto& latest = trajectory.back();
        return CircleCenter(sum_x / total_weight, sum_y / total_weight, 
                          latest.timestamp, latest.t,latest.r,latest.c,latest.d,latest.id);
    }
    
    double calculateDistance(const CircleCenter& a, const CircleCenter& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// 基于几何关系的稳定性增强
class GeometricStabilizer {
private:
    std::deque<std::vector<CircleCenter>> window_;
    size_t min_window_size_ = 3;
    
public:
    std::vector<CircleCenter> stabilizeByGeometry(const std::vector<CircleCenter>& frame) {
        window_.push_back(frame);
        if (window_.size() > min_window_size_ * 2) {
            window_.pop_front();
        }
        
        if (window_.size() < min_window_size_) {
            return frame;
        }
        
        return applyGeometricConstraints();
    }

private:
    std::vector<CircleCenter> applyGeometricConstraints() {
        const auto& latest_frame = window_.back();
        std::vector<CircleCenter> stabilized;
        
        // 计算反光柱之间的相对几何关系
        auto geometric_features = extractGeometricFeatures(latest_frame);
        
        // 与历史帧比较几何一致性
        double consistency_score = checkGeometricConsistency();
        
        if (consistency_score > 0.7) { // 几何关系稳定
            // 使用当前帧
            stabilized = latest_frame;
        } else {
            // 几何关系不稳定，使用历史平均
            stabilized = computeHistoricalAverage();
        }
        
        return stabilized;
    }
    
    // 提取几何特征（距离、角度等）
    std::vector<double> extractGeometricFeatures(const std::vector<CircleCenter>& frame) {
        std::vector<double> features;
        
        if (frame.size() < 2) return features;
        
        // 计算所有点对之间的距离
        for (size_t i = 0; i < frame.size(); ++i) {
            for (size_t j = i + 1; j < frame.size(); ++j) {
                double dist = calculateDistance(frame[i], frame[j]);
                features.push_back(dist);
            }
        }
        
        // 计算角度特征（如果点数足够）
        if (frame.size() >= 3) {
            // 可以计算三角形角度等
        }
        
        return features;
    }
    
    double checkGeometricConsistency() {
        // 比较最近几帧的几何特征一致性
        // 简化实现：返回一致性分数
        return 0.8; // 需要根据实际情况实现
    }
    
    std::vector<CircleCenter> computeHistoricalAverage() {
        // 计算历史帧的平均位置
        std::vector<CircleCenter> average_frame;
        
        // 简化实现：返回最近一帧
        return window_.back();
    }
    
    double calculateDistance(const CircleCenter& a, const CircleCenter& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};
//Gaussain
// 点云数据结构
struct Point {
    double x, y;    // 坐标
    double intensity; // 强度值
    Point() : x(0), y(0), intensity(0){}
    Point(double x, double y) : x(x), y(y), intensity(intensity){}
    
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }
    
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
    
    Point operator/(double scalar) const {
        return Point(x / scalar, y / scalar);
    }
    
    Point& operator+=(const Point& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    double norm() const{
        return std::sqrt(x*x + y*y);
    }
};
struct GaussianParams {
    double A;     // 振幅（圆心处强度）
    double sigma; // 标准差（衰减速度）
};

class ReflectorCenterDetector {
private:
    double expected_radius_;
    double intensity_threshold_;
    
public:
    ReflectorCenterDetector(double radius = 0.03, double threshold = 0.7) 
        : expected_radius_(radius), intensity_threshold_(threshold) {}
    
    bool detectReflectorCenter(const std::vector<Point>& points, Point& center,Point& radar_origin) {
        if (points.empty()) return false;
        
        // 1. 强度阈值过滤
        std::vector<Point> candidates;
        for (const auto& p : points) {
            if (p.intensity > intensity_threshold_) {
                candidates.push_back(p);
            }
        }
        
        if (candidates.size() < 8) {
            //std::cout << "候选点不足: " << candidates.size() << std::endl;
            return false;
        }
        
        // 2. 初始估计：强度加权重心（圆心应该强度最高）
        Point initial_center = computeIntensityWeightedCentroid(candidates);
        //std::cout << "初始估计圆心: (" << initial_center.x << ", " << initial_center.y << ")" << std::endl;
        
        // 3. 基于径向强度衰减的高斯拟合精化
        center = refineCenterWithRadialGaussianFitting(candidates, initial_center);
        
        center = calculateActualCenter(points,center,0.032,radar_origin);
        //std::cout << "精化后圆心: (" << center.x << ", " << center.y << ")" << std::endl;
        
        return true;
    }
    
private:
    // 强度加权重心（圆心处强度最高）
    Point computeIntensityWeightedCentroid(const std::vector<Point>& points) {
        double sum_x = 0, sum_y = 0, sum_weight = 0;
        
        for (const auto& p : points) {
            // 强度越高，权重越大（圆心处强度最高）
            double weight = p.intensity;
            sum_x += p.x * weight;
            sum_y += p.y * weight;
            sum_weight += weight;
        }
        
        Point centroid;
        if (sum_weight > 0) {
            centroid.x = sum_x / sum_weight;
            centroid.y = sum_y / sum_weight;
        } else {
            // 退回到简单平均
            for (const auto& p : points) {
                centroid.x += p.x;
                centroid.y += p.y;
            }
            centroid.x /= points.size();
            centroid.y /= points.size();
        }
        centroid.intensity = 0;
        
        return centroid;
    }
    
    // 基于径向强度衰减的高斯拟合精化圆心
    Point refineCenterWithRadialGaussianFitting(const std::vector<Point>& points, const Point& initial_center) {
        // 收集所有点的径向距离和强度数据
        std::vector<double> distances;
        std::vector<double> intensities;
        
        for (const auto& p : points) {
            double dx = p.x - initial_center.x;
            double dy = p.y - initial_center.y;
            double distance = sqrt(dx * dx + dy * dy);
            
            if (distance < expected_radius_ * 2) { // 只在合理范围内考虑
                distances.push_back(distance);
                intensities.push_back(p.intensity);
            }
        }
        
        if (distances.size() < 5) return initial_center;
        
        // 拟合径向强度分布：I(r) = A * exp(-r^2 / (2 * sigma^2))
        GaussianParams params;
        if (fitRadialGaussian(distances, intensities, params)) {
            //std::cout << "高斯拟合参数: A=" << params.A << ", sigma=" << params.sigma << std::endl;
            
            // 基于拟合结果重新计算圆心（强度最高的区域）
            return recomputeCenterFromGaussianFit(points, initial_center, params);
        }
        
        return initial_center;
    }
    
    // 拟合径向高斯分布：I(r) = A * exp(-r^2 / (2 * sigma^2))
    bool fitRadialGaussian(const std::vector<double>& distances, 
                          const std::vector<double>& intensities,
                          GaussianParams& params) {
        int n = distances.size();
        if (n < 3) return false;
        
        // 转换为线性问题：ln(I) = ln(A) - r^2 / (2 * sigma^2)
        std::vector<double> r_squared;
        std::vector<double> log_intensities;
        
        for (int i = 0; i < n; i++) {
            if (intensities[i] > 1e-6) { // 避免log(0)
                r_squared.push_back(distances[i] * distances[i]);
                log_intensities.push_back(log(intensities[i]));
            }
        }
        
        if (r_squared.size() < 3) return false;
        
        // 线性回归：y = a + b*x, 其中 y = ln(I), x = r^2
        // b = -1/(2*sigma^2), a = ln(A)
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        int m = r_squared.size();
        
        for (int i = 0; i < m; i++) {
            sum_x += r_squared[i];
            sum_y += log_intensities[i];
            sum_xy += r_squared[i] * log_intensities[i];
            sum_x2 += r_squared[i] * r_squared[i];
        }
        
        double denominator = m * sum_x2 - sum_x * sum_x;
        if (fabs(denominator) < 1e-10) return false;
        
        double b = (m * sum_xy - sum_x * sum_y) / denominator;
        double a = (sum_y - b * sum_x) / m;
        
        // 提取高斯参数
        params.A = exp(a);
        if (b >= 0) return false; // b应该是负值
        params.sigma = sqrt(-1.0 / (2 * b));
        
        return true;
    }
    
    // 基于高斯拟合结果重新计算圆心
    Point recomputeCenterFromGaussianFit(const std::vector<Point>& points, 
                                       const Point& current_center,
                                       const GaussianParams& params) {
        // 方法1：寻找使总似然最大的圆心位置
        double best_x = current_center.x;
        double best_y = current_center.y;
        double best_likelihood = -std::numeric_limits<double>::max();
        
        // 在当前圆心周围搜索最优位置
        const double search_range = 0.02; // 2cm搜索范围
        const int search_steps = 20;
        
        for (int i = -search_steps; i <= search_steps; i++) {
            for (int j = -search_steps; j <= search_steps; j++) {
                double test_x = current_center.x + i * search_range / search_steps;
                double test_y = current_center.y + j * search_range / search_steps;
                
                double likelihood = computeGaussianLikelihood(points, test_x, test_y, params);
                
                if (likelihood > best_likelihood) {
                    best_likelihood = likelihood;
                    best_x = test_x;
                    best_y = test_y;
                }
            }
        }
        
        Point refined_center;
        refined_center.x = best_x;
        refined_center.y = best_y;
        refined_center.intensity = params.A;
        
        /*std::cout << "似然最大化搜索: 从(" << current_center.x << "," << current_center.y 
                  << ") 到 (" << best_x << "," << best_y << ")" << std::endl;*/
        
        return refined_center;
    }
    
    // 计算给定圆心位置的高斯似然
    double computeGaussianLikelihood(const std::vector<Point>& points, 
                                   double center_x, double center_y,
                                   const GaussianParams& params) {
        double total_log_likelihood = 0;
        int count = 0;
        
        for (const auto& p : points) {
            double dx = p.x - center_x;
            double dy = p.y - center_y;
            double distance = sqrt(dx * dx + dy * dy);
            
            if (distance < expected_radius_ * 3) { // 合理范围内
                // 高斯模型预测的强度
                double predicted_intensity = params.A * exp(-distance * distance / (2 * params.sigma * params.sigma));
                
                if (predicted_intensity > 1e-6) {
                    // 使用对数似然：假设观测强度围绕预测值呈高斯分布
                    double error = p.intensity - predicted_intensity;
                    total_log_likelihood += -error * error; // 负的误差平方和
                    count++;
                }
            }
        }
        
        return count > 0 ? total_log_likelihood / count : -1e10;
    }
    // 新增方法：从反射中心计算实际圆心
    // 新增方法：从反射中心计算实际圆心（在反射中心-雷达连线的延长线上）
    Point calculateActualCenter(const std::vector<Point>& points, const Point& reflection_center, 
                            double expected_radius, const Point& radar_origin) {
        Point actual_center = reflection_center; // 默认值
        
        // 计算反射中心到雷达原点的方向向量
        double dx = reflection_center.x - radar_origin.x;
        double dy = reflection_center.y - radar_origin.y;
        double distance_to_radar = sqrt(dx * dx + dy * dy);
        
        if (distance_to_radar < 1e-10) {
            std::cout << "警告：反射中心与雷达原点太近" << std::endl;
            return actual_center;
        }
        
        // 单位方向向量（从雷达指向反射中心）
        double dir_x = dx / distance_to_radar;
        double dir_y = dy / distance_to_radar;
        
        /*std::cout << "雷达原点: (" << radar_origin.x << ", " << radar_origin.y << ")" << std::endl;
        std::cout << "反射中心: (" << reflection_center.x << ", " << reflection_center.y << ")" << std::endl;
        std::cout << "方向向量: (" << dir_x << ", " << dir_y << ")" << std::endl;
        std::cout << "距离雷达: " << distance_to_radar << " m" << std::endl;*/
        
        // 实际圆心在反射中心-雷达连线的延长线上，距离反射中心一个半径
        // 因为圆心在圆柱的另一侧，所以方向与雷达->反射中心方向相同
        actual_center.x = reflection_center.x + dir_x * expected_radius;
        actual_center.y = reflection_center.y + dir_y * expected_radius;
        actual_center.intensity = 0;
        
        /*std::cout << "实际圆心: (" << actual_center.x << ", " << actual_center.y << ")" << std::endl;
        std::cout << "偏移距离: " << expected_radius << " m" << std::endl;*/
        
        return actual_center;
    }
};
// 纯 C++ 数据结构，替代 ROS 类型
struct LaserScan
{
  std::string header = "";
  std::vector<float> ranges;
  std::vector<float> intensities;
  float angle_min = 0.0f;
  float angle_max = 0.0f;
  float angle_increment = 0.0f;

  float time_increment = 0.0f;
  float scan_time = 0.0f; // 这个scan_time表示什么？

  float range_min = 0.0f;
  float range_max = 0.0f;
};

class ReflectivePostDetector
{
public:
  ReflectivePostDetector(int intensity_threshold = 1000);
  ~ReflectivePostDetector();

  // Detect reflective posts from laser scan (非 ROS 环境可直接调用)
  std::vector<ReflectivePost> detect(const LaserScan & scan);
  std::vector<Detection> detect_circles(const LaserScan & scan);


  // 参数声明
  int intensity_threshold_use = 1000;
  double cluster_eps = 0.064;
  int min_cluster_points = 3;
  double diameter_min = 0.04;
  double diameter_max = 0.08;
  double residual_avg_threshold = 0.01;
  double residual_std_threshold = 0.005;
  double residual_max_threshold = 0.02;
  int stat_mean_k = 4;
  double stat_std_threshold = 1.0;
  int max_history_age = 3;
  double match_distance_threshold = 0.1;
  double arc_threshold = 0.1;
  double max_arc_feature = 30.0;
  int arc_min_points = 4;
  // double direction_tolerance = 0.785;
  double residual_real = 0.032;
  double sensitivity = 2.0;
  double maxError = 1.0;
  double maxangleError = 0.1;
  double paramError = 0.001;
  double distance_decay = 3.0;
  double convergence_threshold = 1e-6;
  int max_iterations = 1000;
  double residual_tolerance = 0.02;
  double landmark_rotation_weight = 1e2;
  double landmark_translation_weight = 1e5;

private:
  RelativeCircleSmoother smoother;
  GeometricStabilizer geometric_stabilizer;
  // 定义基本数据结构
  struct WeightedCircle {
      double center_x, center_y;
      double weight_translation;  // 平移权重
      double weight_rotation;     // 旋转权重
      double fit_error;           // 拟合误差
      int point_count;            // 使用点数
  };
  struct WeightParams {
      double ideal_point_count = 5.0;    // 理想点数
      double error_threshold = 0.001;    // 误差阈值
      double distance_decay = 10.0;      // 距离衰减系数
      double min_angle_diversity = 0.5;  // 最小角度多样性阈值
  };
  /**
  * 计算角度多样性指标 (0-1之间)
  */
  double calculateAngleDiversity(const std::vector<Point>& points, 
                                  const Point& center) {
      if (points.size() < 3) return 0.5; // 点数太少，给中等权重
      
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
  * 为计算出的圆心赋权重
  */
  WeightedCircle assignWeightsToCircle(
      const std::vector<Point>& points,      // 使用的点云
      const Point& calculated_center,        // 计算出的圆心
      double fit_error,                      // 拟合误差
      const Point& vehicle_position,         // 车辆当前位置（用于距离计算）
      const WeightParams& params) {
      
      WeightedCircle result;
      result.center_x = calculated_center.x;
      result.center_y = calculated_center.y;
      result.fit_error = fit_error;
      result.point_count = points.size();
      
      // 计算到车辆的距离
      double dx = calculated_center.x - vehicle_position.x;
      double dy = calculated_center.y - vehicle_position.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      // 1. 计算各分量权重
      double W_count = std::min(static_cast<double>(points.size()) / 
                              params.ideal_point_count, 1.0);
      
      double W_error = std::exp(-fit_error / params.error_threshold);
      
      double W_distance = std::exp(-distance / params.distance_decay);
      
      double W_angle = calculateAngleDiversity(points, calculated_center);
      
      // 2. 组合最终权重
      result.weight_translation = W_count * W_error * W_distance;
      
      // 旋转权重更注重角度多样性和距离
      result.weight_rotation = W_angle * W_distance;
      
      // 确保权重在合理范围内
      result.weight_translation = std::clamp(result.weight_translation, 0.0, 1.0);
      result.weight_rotation = std::clamp(result.weight_rotation, 0.0, 1.0);
      
      return result;
  }
  // 统计离群点过滤
    std::vector<Point> statistical_outlier_filter(const std::vector<Point>& points)
    {
        std::vector<double> mean_distances;
        for (const auto& p : points) {
            std::vector<double> distances;
            for (const auto& other : points) {
                if (&p != &other) {
                    distances.push_back((p - other).norm());
                }
            }
            std::sort(distances.begin(), distances.end());
            double mean_dist = 0.0;
            int count = std::min(stat_mean_k, static_cast<int>(distances.size()));
            for (int i = 0; i < count; ++i) {
                mean_dist += distances[i];
            }
            mean_dist /= count;
            mean_distances.push_back(mean_dist);
        }

        double mean = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0) / mean_distances.size();
        double std_dev = 0.0;
        for (double d : mean_distances) {
            std_dev += (d - mean) * (d - mean);
        }
        std_dev = sqrt(std_dev / mean_distances.size());

        std::vector<Point> filtered;
        for (size_t i = 0; i < points.size(); ++i) {
            if (mean_distances[i] < mean + stat_std_threshold * std_dev) {
                filtered.push_back(points[i]);
            }
        }
        return filtered;
    }
    // 计算两点之间的欧氏距离
    double distanceTo(const Point& current_points,const Point& other){
        double dx = current_points.x - other.x;
        double dy = current_points.y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    // 计算点集的 k 近邻距离
	std::vector<double> compute_knn_distances(const std::vector<Point>& points, int k) {
	    std::vector<double> avg_distances(points.size(), 0.0);
	    
	    for (size_t i = 0; i < points.size(); i++) {
			std::vector<double> distances;
			
			// 计算当前点到所有其他点的距离
			for (size_t j = 0; j < points.size(); j++) {
				if (i != j) {
				    distances.push_back(distanceTo(points[i],points[j]));
				}
			}
			
			// 排序距离
			std::sort(distances.begin(), distances.end());
			
			// 取前 k 个最小距离的平均值
			double sum = 0.0;
			int count = std::min(k, static_cast<int>(distances.size()));
			for (int idx = 0; idx < count; idx++) {
				sum += distances[idx];
			}
			
			avg_distances[i] = sum / count;
	    }
	    
	    return avg_distances;
	}

	// 计算向量的中位数
	double compute_median(std::vector<double> values) {
	    if (values.empty()) {
		return 0.0;
	    }
	    
	    std::sort(values.begin(), values.end());
	    size_t n = values.size();
	    
	    if (n % 2 == 0) {
		return (values[n/2 - 1] + values[n/2]) / 2.0;
	    } else {
		return values[n/2];
	    }
	}

	// DBSCAN 聚类算法实现
	std::vector<int> dbscan(const std::vector<Point>& points, double eps, int min_samples) {
		std::vector<int> labels(points.size(), -1); // -1 表示噪声点
		int cluster_id = 0;
		
		for (size_t i = 0; i < points.size(); i++) {
		    if (labels[i] != -1) {
		        continue; // 已经处理过的点
		    }
		    
		    // 找到当前点的邻域点
		    std::vector<size_t> neighbors;
		    for (size_t j = 0; j < points.size(); j++) {
		        if (i != j && distanceTo(points[i],points[j]) <= eps) {
		            neighbors.push_back(j);
		        }
		    }
		    
		    // 检查是否为核心点
		    if (int(neighbors.size()) < min_samples) {
		        labels[i] = -1; // 标记为噪声点
		        continue;
		    }
		    
		    // 开始新的聚类
		    cluster_id++;
		    labels[i] = cluster_id;
		    
		    // 使用队列扩展聚类
		    std::queue<size_t> cluster_queue;
		    for (size_t neighbor : neighbors) {
		        cluster_queue.push(neighbor);
		    }
		    
		    while (!cluster_queue.empty()) {
		        size_t current_idx = cluster_queue.front();
		        cluster_queue.pop();
		        
		        if (labels[current_idx] == -1) {
		            labels[current_idx] = cluster_id;
		        } else if (labels[current_idx] != 0) {
		            continue; // 已经处理过的点
		        }
		        
		        labels[current_idx] = cluster_id;
		        
		        // 找到当前点的邻域点
		        std::vector<size_t> current_neighbors;
		        for (size_t j = 0; j < points.size(); j++) {
		            if (current_idx != j && distanceTo(points[current_idx],points[j]) <= eps) {
		                current_neighbors.push_back(j);
		            }
		        }
		        
		        // 如果当前点也是核心点，将其邻域点加入队列
		        if (int(current_neighbors.size()) >= min_samples) {
		            for (size_t neighbor : current_neighbors) {
		                if (labels[neighbor] == -1 || labels[neighbor] == 0) {
		                    cluster_queue.push(neighbor);
		                }
		            }
		        }
		    }
		}
		
		return labels;
	}

	// 自适应 DBSCAN 聚类
	std::vector<int> adaptive_dbscan(const std::vector<Point>& points) {
		if (int(points.size()) < min_cluster_points) {
		    return std::vector<int>(points.size(), -1);
		}
		
		// 计算 k 近邻距离
		int k = 4;
		std::vector<double> avg_distances = compute_knn_distances(points, k);
		
		// 计算中位数
		double median_eps = compute_median(avg_distances);
		
		// 确定最终的 eps 值
		double eps = std::max(cluster_eps, median_eps);
		//RCLCPP_INFO(this->get_logger(), "info eps: %f", eps);
		// 执行 DBSCAN 聚类
		return dbscan(points, eps, min_cluster_points);
	}
};

} // namespace landmark_localization
