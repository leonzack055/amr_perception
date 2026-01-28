#pragma once

#include "amr_reflector_noise_handling/types.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <stack>
#include <vector>

namespace amr_reflector_noise_handling {

/**
 * @brief 圆拟合结果
 */
struct CircleFitResult {
  Point center;
  double radius;
  double fit_error;     // 拟合误差（RMS）
  double outline_error; // 外点平均误差（RMS）
  double inner_error;   // 内点的平均误差
  int inlier_count;     // 内点数量
  int total_points;     // 总点数
  double inlier_ratio;  // 内点比例
  bool is_valid;        // 是否有效
};

/**
 * @brief 凸性指标
 */
struct ConvexityMetrics {
  double convexity_ratio;    // 凸包面积 / 点云面积
  bool is_convex;            // 是否完全凸
  double angular_span;       // 角度跨度（弧度）
  double curvature_variance; // 曲率方差
  double hull_area;          // 凸包面积
  double cloud_area;         // 点云面积
};

/**
 * @brief 圆拟合参数
 */
struct CircleFitParams {
  double max_fit_error = 0.03;         // 最大拟合误差3cm
  double min_inlier_ratio = 0.56;      // 最小内点比例50%
  double max_fit_error_near = 0.003;   // 近距离最大误差4cm
  double max_fit_error_far = 0.004;    // 远距离最大误差2cm
  double far_distance_threshold = 3.0; // 远距离阈值3m
  double min_radius = 0.02;            // 最小半径2cm
  double max_radius = 0.05;            // 最大半径5cm
  double inlier_threshold = 0.02;      // 内点阈值2cm

  // 凸性检测参数
  double min_convexity_ratio = 0.7;       // 最小凸性比率
  double min_angular_span = M_PI / 6;     // 最小角度跨度30度
  double max_angular_span = 2 * M_PI / 3; // 最大角度跨度120度
  double max_curvature_variance = 0.5;    // 最大曲率方差

  // RANSAC参数
  int ransac_iterations = 100;             // RANSAC迭代次数
  double ransac_inlier_threshold = 0.0015; // RANSAC内点阈值
};

/**
 * @brief 圆拟合器
 *
 * 对插值后的点云进行圆拟合，用于验证反光柱候选
 * 支持部分圆弧拟合，利用凸性、曲率等特征区分圆弧和团状噪声
 */
class CircleFitter {
public:
  CircleFitter() = default;

  /**
   * @brief 设置拟合参数
   */
  void setParams(const CircleFitParams &params) { params_ = params; }

private:
  CircleFitParams params_;

public:
  /**
   * @brief 鲁棒圆弧拟合（主入口）
   *
   * 结合凸性检测、RANSAC拟合和改进验证
   *  此处凸包计算存在问题，
   * @param points 点云
   * @param distance 距离（用于自适应阈值）
   * @return CircleFitResult 拟合结果
   */
  CircleFitResult robustArcFitting(const std::vector<Point> &points,
                                   double distance) const {
    CircleFitResult result;
    result.is_valid = false;
    result.total_points = points.size();

    if (points.size() < 5) {
      return result;
    }

    // 1. 计算凸性指标
    auto metrics = computeConvexity(points);

    // 2. 快速拒绝：凸性比率太低，可能是团状噪声
    if (metrics.convexity_ratio < params_.min_convexity_ratio) {
      return result;
    }

    // 3. 使用RANSAC进行鲁棒拟合
    auto ransac_result = fitArcWithRANSAC(points);

    if (!ransac_result.is_valid) {
      return result;
    }

    // 4. 使用内点重新拟合（更精确）
    std::vector<Point> inliers;
    for (const auto &p : points) {
      if (std::abs(p.distanceTo(ransac_result.center) - ransac_result.radius) <
          params_.ransac_inlier_threshold) {
        inliers.push_back(p);
      }
    }

    if (inliers.size() < 3) {
      return result;
    }

    auto refined_result = fitCircleFromInliers(inliers);

    // 5. 验证拟合结果
    if (!validateArcFit(refined_result, metrics, points.size(), distance)) {
      refined_result.is_valid = false;
    }

    return refined_result;
  }

  /**
   * @brief 计算凸性指标
   * WARN: 凸包计算过程中存在问题，应该以points中索引顺序进行凸包计算;
   * 角度跨度应该以拟合圆中心计算
   * @param points 点云
   * @return ConvexityMetrics 凸性指标
   */
  ConvexityMetrics computeConvexity(const std::vector<Point> &points) const {
    ConvexityMetrics metrics;

    if (points.size() < 3) {
      metrics.is_convex = false;
      metrics.convexity_ratio = 0.0;
      return metrics;
    }

    // 1. 计算凸包
    auto convex_hull = computeConvexHull(points);

    // 2. 计算凸包面积
    metrics.hull_area = computePolygonArea(convex_hull);

    // 3. 计算点云面积（使用凸包近似）
    metrics.cloud_area = metrics.hull_area;

    // 4. 判断是否完全凸
    metrics.is_convex = (convex_hull.size() == points.size());

    // 5. 计算凸性比率
    if (metrics.cloud_area > 1e-6) {
      metrics.convexity_ratio = metrics.hull_area / metrics.cloud_area;
    } else {
      metrics.convexity_ratio = 0.0;
    }

    // 6. 计算角度跨度
    Point center = computeCentroid(points);
    std::vector<double> angles;
    for (const auto &p : points) {
      double angle = std::atan2(p.y - center.y, p.x - center.x);
      angles.push_back(angle);
    }
    std::sort(angles.begin(), angles.end());

    // 处理跨越-π/π的情况
    if (angles.back() - angles.front() > M_PI) {
      // 跨越了-π/π边界，需要特殊处理
      double span1 = 2 * M_PI - angles.back() + angles.front();
      double span2 = angles.back() - angles.front();
      metrics.angular_span = std::min(span1, span2);
    } else {
      metrics.angular_span = angles.back() - angles.front();
    }

    // 7. 计算曲率方差
    metrics.curvature_variance = computeCurvatureVariance(points);

    return metrics;
  }

  /**
   * @brief RANSAC圆弧拟合
   *
   * @param points 点云
   * @return CircleFitResult 拟合结果
   */
  CircleFitResult fitArcWithRANSAC(const std::vector<Point> &points) const {
    CircleFitResult best_result;
    best_result.fit_error = std::numeric_limits<double>::max();
    best_result.is_valid = false;
    best_result.total_points = points.size();
    best_result.inlier_count = 0;

    if (points.size() < 13) {
      std::cout << "拟合圆点数少于15个点" << std::endl;
      return best_result;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int iter = 0; iter < params_.ransac_iterations; ++iter) {
      // 1. 随机选择3个点
      int i1 = dis(gen);
      int i2 = dis(gen);
      int i3 = dis(gen);
      while (i2 == i1)
        i2 = dis(gen);
      while (i3 == i1 || i3 == i2)
        i3 = dis(gen);

      // 2. 通过三点确定圆
      auto circle = fitCircleFrom3Points(points[i1], points[i2], points[i3]);

      if (!circle.is_valid) {
        continue;
      }

      // 3. 计算内点
      int inlier_count = 0;
      int outlier_count = 0;
      double error_sum = 0;
      double inerror_sum = 0;
      double outerror_sum = 0;
      for (const auto &p : points) {
        double residual = std::abs(p.distanceTo(circle.center) - circle.radius);
        // 统计总体误差，看总体平均误差范围: fit_error拟合点的误差;
        // outline_error离群点的误差； mean_error均值误差
        if (residual < params_.ransac_inlier_threshold) {
          inlier_count++;
          inerror_sum += residual * residual;
        } else {
          outlier_count++;
          outerror_sum += residual * residual;
        }
      }

      // 4. 评估拟合质量
      double inlier_ratio = static_cast<double>(inlier_count) / points.size();
      error_sum = inerror_sum + outerror_sum;
      double fit_error = std::sqrt(error_sum / points.size());
      double inner_error = std::sqrt(inerror_sum / inlier_count);
      double outlier_error = std::sqrt(outerror_sum / outlier_count);

      // 5. 更新最佳结果
      if (inlier_count > best_result.inlier_count ||
          (inlier_count == best_result.inlier_count &&
           fit_error < best_result.fit_error)) {
        best_result.center = circle.center;
        best_result.radius = circle.radius;
        best_result.inlier_count = inlier_count;
        best_result.inlier_ratio = inlier_ratio;
        best_result.inner_error = inner_error;
        best_result.outline_error = outlier_error;
        best_result.fit_error = fit_error;
        best_result.is_valid = true;
      }
    }

    return best_result;
  }

  /**
   * @brief 验证圆弧拟合结果
   *
   * @param fit 拟合结果
   * @param metrics 凸性指标
   * @param original_point_count 原始点数
   * @param distance 距离
   * @return bool 是否有效
   */
  bool validateArcFit(const CircleFitResult &fit,
                      const ConvexityMetrics &metrics,
                      [[maybe_unused]] int original_point_count,
                      double distance) const {

    // 1. 基本验证（半径）
    if (fit.radius < params_.min_radius || fit.radius > params_.max_radius) {
      return false;
    }

    // 2. 拟合误差检查（根据距离自适应）
    double max_error = params_.max_fit_error;
    if (distance < params_.far_distance_threshold) {
      max_error = params_.max_fit_error_near;
    } else {
      max_error = params_.max_fit_error_far;
    }

    if (fit.fit_error > max_error) {
      return false;
    }

    // 3. 凸性检查（区分圆弧和团状噪声）
    if (metrics.convexity_ratio < params_.min_convexity_ratio) {
      return false;
    }

    // 4. 角度跨度检查（确保有足够的圆弧）
    if (metrics.angular_span < params_.min_angular_span ||
        metrics.angular_span > params_.max_angular_span) {
      return false;
    }

    // 5. 曲率一致性检查
    if (metrics.curvature_variance > params_.max_curvature_variance) {
      return false;
    }

    // 6. 内点比例
    double expected_inlier_ratio = params_.min_inlier_ratio;
    double actual_inlier_ratio = fit.inlier_ratio;
    if (actual_inlier_ratio < expected_inlier_ratio) {
      return false;
    }

    return true;
  }

  /**
   * @brief 简单迭代圆拟合（保留用于兼容）
   *
   * @param points 点云
   * @return CircleFitResult 拟合结果
   */
  CircleFitResult fitCircle(const std::vector<Point> &points) const {
    CircleFitResult result;
    result.total_points = points.size();
    result.is_valid = false;

    if (points.size() < 3) {
      result.fit_error = 999;
      return result;
    }

    // 1. 计算初始中心（简单平均）
    double sum_x = 0, sum_y = 0;
    for (const auto &p : points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    result.center.x = sum_x / points.size();
    result.center.y = sum_y / points.size();

    // 2. 迭代优化（5次迭代足够）
    for (int iter = 0; iter < 5; ++iter) {
      // 2.1 计算每个点到中心的距离
      std::vector<double> distances;
      for (const auto &p : points) {
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
    for (const auto &p : points) {
      double d = p.distanceTo(result.center);
      all_distances.push_back(d);
    }

    result.radius =
        std::accumulate(all_distances.begin(), all_distances.end(), 0.0) /
        all_distances.size();

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
    result.inlier_ratio =
        static_cast<double>(result.inlier_count) / points.size();

    return result;
  }

  /**
   * @brief 验证圆拟合结果（保留用于兼容）
   *
   * @param fit 拟合结果
   * @param original_point_count 原始点数
   * @param distance 距离
   * @return bool 是否有效
   */
  bool validateFit(const CircleFitResult &fit, int original_point_count,
                   double distance) const {

    // 1. 拟合误差检查
    double max_error = params_.max_fit_error;

    // 根据距离调整误差阈值
    if (distance < params_.far_distance_threshold) {
      max_error = params_.max_fit_error_near; // 近距离允许更大误差fit_error
    } else {
      max_error = params_.max_fit_error_far; // 远距离要求更小误差
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
    double expected_points = original_point_count;
    double expected_inlier_ratio = params_.min_inlier_ratio;

    double actual_inlier_ratio =
        static_cast<double>(fit.inlier_count) / expected_points;
    if (actual_inlier_ratio < expected_inlier_ratio) {
      return false;
    }

    return true;
  }

private:
  /**
   * @brief 计算凸包（Graham扫描算法）
   *
   * @param points 点云
   * @return std::vector<Point> 凸包点
   */
  std::vector<Point> computeConvexHull(const std::vector<Point> &points) const {
    if (points.size() < 3) {
      return points;
    }

    // 1. 找到y坐标最小的点作为起点
    size_t start_idx = 0;
    for (size_t i = 1; i < points.size(); ++i) {
      if (points[i].y < points[start_idx].y ||
          (points[i].y == points[start_idx].y &&
           points[i].x < points[start_idx].x)) {
        start_idx = i;
      }
    }

    Point start = points[start_idx];

    // 2. 按极角排序
    std::vector<Point> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(),
              [&start](const Point &a, const Point &b) {
                double angle_a = std::atan2(a.y - start.y, a.x - start.x);
                double angle_b = std::atan2(b.y - start.y, b.x - start.x);
                if (std::abs(angle_a - angle_b) > 1e-6) {
                  return angle_a < angle_b;
                }
                // 角度相同，按距离排序
                double dist_a = std::sqrt((a.x - start.x) * (a.x - start.x) +
                                          (a.y - start.y) * (a.y - start.y));
                double dist_b = std::sqrt((b.x - start.x) * (b.x - start.x) +
                                          (b.y - start.y) * (b.y - start.y));
                return dist_a < dist_b;
              });

    // 3. Graham扫描构建凸包
    std::vector<Point> hull;
    hull.push_back(sorted_points[0]);
    hull.push_back(sorted_points[1]);

    for (size_t i = 2; i < sorted_points.size(); ++i) {
      while (hull.size() >= 2) {
        Point p1 = hull[hull.size() - 2];
        Point p2 = hull[hull.size() - 1];
        Point p3 = sorted_points[i];

        // 计算叉积
        double cross =
            (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

        if (cross <= 0) {
          hull.pop_back();
        } else {
          break;
        }
      }
      hull.push_back(sorted_points[i]);
    }

    return hull;
  }

  /**
   * @brief 计算多边形面积
   *
   * @param polygon 多边形顶点
   * @return double 面积
   */
  double computePolygonArea(const std::vector<Point> &polygon) const {
    if (polygon.size() < 3) {
      return 0.0;
    }

    double area = 0.0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;
      area += polygon[i].x * polygon[j].y;
      area -= polygon[j].x * polygon[i].y;
    }

    return std::abs(area) / 2.0;
  }

  /**
   * @brief 计算质心
   *
   * @param points 点云
   * @return Point 质心
   */
  Point computeCentroid(const std::vector<Point> &points) const {
    Point centroid;
    if (points.empty()) {
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &point : points) {
      sum_x += point.x;
      sum_y += point.y;
    }

    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    return centroid;
  }

  /**
   * @brief 计算曲率方差
   *
   * @param points 点云
   * @return double 曲率方差
   */
  double computeCurvatureVariance(const std::vector<Point> &points) const {
    if (points.size() < 3) {
      return 0.0;
    }

    std::vector<double> curvatures;

    for (size_t i = 1; i < points.size() - 1; ++i) {
      // 计算相邻三点形成的角度
      const Point &p1 = points[i - 1];
      const Point &p2 = points[i];
      const Point &p3 = points[i + 1];

      // 计算向量
      double v1x = p1.x - p2.x;
      double v1y = p1.y - p2.y;
      double v2x = p3.x - p2.x;
      double v2y = p3.y - p2.y;

      // 计算夹角（曲率的近似）
      double dot = v1x * v2x + v1y * v2y;
      double norm1 = std::sqrt(v1x * v1x + v1y * v1y);
      double norm2 = std::sqrt(v2x * v2x + v2y * v2y);

      if (norm1 > 1e-6 && norm2 > 1e-6) {
        double cos_angle = std::clamp(dot / (norm1 * norm2), -1.0, 1.0);
        double angle = std::acos(cos_angle);
        curvatures.push_back(angle);
      }
    }

    if (curvatures.empty()) {
      return 0.0;
    }

    // 计算平均曲率
    double mean_curvature =
        std::accumulate(curvatures.begin(), curvatures.end(), 0.0) /
        curvatures.size();

    // 计算曲率方差
    double variance = 0.0;
    for (double c : curvatures) {
      variance += (c - mean_curvature) * (c - mean_curvature);
    }
    variance /= curvatures.size();

    return variance;
  }

  /**
   * @brief 通过三点确定圆
   *
   * @param p1 第一个点
   * @param p2 第二个点
   * @param p3 第三个点
   * @return CircleFitResult 圆拟合结果
   */
  CircleFitResult fitCircleFrom3Points(const Point &p1, const Point &p2,
                                       const Point &p3) const {
    CircleFitResult result;
    result.is_valid = false;

    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double x3 = p3.x, y3 = p3.y;

    double D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    if (std::abs(D) < 1e-6) {
      return result; // 三点共线
    }

    double Ux =
        ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
         (x3 * x3 + y3 * y3) * (y1 - y2)) /
        D;

    double Uy =
        ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
         (x3 * x3 + y3 * y3) * (x2 - x1)) /
        D;

    result.center.x = Ux;
    result.center.y = Uy;
    result.radius = std::sqrt((x1 - Ux) * (x1 - Ux) + (y1 - Uy) * (y1 - Uy));
    result.is_valid = true;

    return result;
  }

  /**
   * @brief 使用内点拟合圆（最小二乘）
   *
   * @param inliers 内点
   * @return CircleFitResult 拟合结果
   */
  CircleFitResult
  fitCircleFromInliers(const std::vector<Point> &inliers) const {
    CircleFitResult result;
    result.is_valid = false;
    result.total_points = inliers.size();

    if (inliers.size() < 3) {
      return result;
    }

    // 使用Kasa方法进行最小二乘圆拟合
    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_yy = 0, sum_xy = 0;
    double sum_xxx = 0, sum_xyy = 0, sum_yxx = 0, sum_yyy = 0;

    for (const auto &p : inliers) {
      double x = p.x;
      double y = p.y;
      sum_x += x;
      sum_y += y;
      sum_xx += x * x;
      sum_yy += y * y;
      sum_xy += x * y;
      sum_xxx += x * x * x;
      sum_xyy += x * y * y;
      sum_yxx += y * x * x;
      sum_yyy += y * y * y;
    }

    int n = inliers.size();

    // 构建线性方程组
    double A[3][3] = {{sum_xx, sum_xy, sum_x},
                      {sum_xy, sum_yy, sum_y},
                      {sum_x, sum_y, static_cast<double>(n)}};

    double B[3] = {-0.5 * (sum_xxx + sum_xyy), -0.5 * (sum_yxx + sum_yyy),
                   -0.5 * (sum_xx + sum_yy)};

    // 高斯消元法求解
    for (int i = 0; i < 3; ++i) {
      int pivot = i;
      for (int j = i + 1; j < 3; ++j) {
        if (std::abs(A[j][i]) > std::abs(A[pivot][i])) {
          pivot = j;
        }
      }

      std::swap(A[i], A[pivot]);
      std::swap(B[i], B[pivot]);

      if (std::abs(A[i][i]) < 1e-10) {
        return result;
      }

      for (int j = i + 1; j < 3; ++j) {
        double factor = A[j][i] / A[i][i];
        for (int k = i; k < 3; ++k) {
          A[j][k] -= factor * A[i][k];
        }
        B[j] -= factor * B[i];
      }
    }

    // 回代求解
    double x[3];
    for (int i = 2; i >= 0; --i) {
      x[i] = B[i];
      for (int j = i + 1; j < 3; ++j) {
        x[i] -= A[i][j] * x[j];
      }
      x[i] /= A[i][i];
    }

    result.center.x = x[0];
    result.center.y = x[1];
    result.radius = std::sqrt(x[0] * x[0] + x[1] * x[1] - x[2]);
    result.is_valid = true;

    // 计算拟合误差
    double error_sum = 0;
    result.inlier_count = 0;
    for (const auto &p : inliers) {
      double d = p.distanceTo(result.center);
      double error = std::abs(d - result.radius);
      error_sum += error * error;
      if (std::abs(d - result.radius) < params_.inlier_threshold) {
        result.inlier_count++;
      }
    }
    result.fit_error = std::sqrt(error_sum / inliers.size());
    result.inlier_ratio =
        static_cast<double>(result.inlier_count) / inliers.size();

    return result;
  }
};

} // namespace amr_reflector_noise_handling