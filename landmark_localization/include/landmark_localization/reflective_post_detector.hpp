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
#include "landmark_localization/common/reflector_common.hpp"
namespace landmark_localization
{

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
  float scan_time = 0.0f;

  float range_min = 0.0f;
  float range_max = 0.0f;
};

class ReflectivePostDetector
{
public:
  ReflectivePostDetector(int intensity_threshold = 1600);
  ~ReflectivePostDetector();

  // Detect reflective posts from laser scan (非 ROS 环境可直接调用)
  std::vector<ReflectivePost> detect(const LaserScan & scan);

private:
  // 参数声明
  int intensity_threshold_use = 1600;
  double cluster_eps = 0.064;
  int min_cluster_points = 12;
  double diameter_min = 0.05;
  double diameter_max = 0.15;
  double residual_avg_threshold = 0.01;
  double residual_std_threshold = 0.005;
  double residual_max_threshold = 0.02;
  int stat_mean_k = 4;
  double stat_std_threshold = 1.0;
  int max_history_age = 3;
  double match_distance_threshold = 0.1;
  double arc_threshold = 0.1;
  double max_arc_feature = 30.0;
  int arc_min_points = 10;
  // double direction_tolerance = 0.785;
  double residual_real = 0.032;
  double sensitivity = 2.0;
  double maxError = 1.0;
  double maxangleError = 0.1;
  double landmark_rotation_weight = 1e2;
  double landmark_translation_weight = 1e5;

  // 定义基本数据结构
  struct Point
  {
    double x, y, z;

    Point()
    : x(0), y(0), z(0) {}
    Point(double x, double y, double z)
    : x(x), y(y), z(z) {}

    // 计算两点间距离
    double distanceTo(const Point & other) const
    {
      double dx = x - other.x;
      double dy = y - other.y;
      double dz = z - other.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
  };

  struct Vector2d
  {
    double x, y;
    Vector2d()
    : x(0), y(0) {}
    Vector2d(double x, double y)
    : x(x), y(y) {}

    Vector2d operator+(const Vector2d & other) const
    {
      return Vector2d(x + other.x, y + other.y);
    }

    Vector2d operator-(const Vector2d & other) const
    {
      return Vector2d(x - other.x, y - other.y);
    }

    Vector2d operator/(double scalar) const
    {
      return Vector2d(x / scalar, y / scalar);
    }

    Vector2d & operator+=(const Vector2d & other)
    {
      x += other.x;
      y += other.y;
      return *this;
    }
    double norm() const
    {
      return std::sqrt(x * x + y * y);
    }
  };

  double get_primary_direction(const std::vector<Vector2d> & points)
  {
    if (points.size() < 2) {return 0.0;}

    // 计算均值
    Vector2d mean;
    for (const auto & p : points) {
      mean += p;
    }
    mean = mean / static_cast<double>(points.size());

    // 计算协方差矩阵
    double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
    for (const auto & p : points) {
      Vector2d centered = {p.x - mean.x, p.y - mean.y};
      cov_xx += centered.x * centered.x;
      cov_xy += centered.x * centered.y;
      cov_yy += centered.y * centered.y;
    }

    double n = static_cast<double>(points.size());
    cov_xx /= n;
    cov_xy /= n;
    cov_yy /= n;

    // 计算特征值和特征向量
    double trace = cov_xx + cov_yy;
    double determinant = cov_xx * cov_yy - cov_xy * cov_xy;

    // 计算特征值
    double eigenvalue1 = trace / 2.0 + std::sqrt(trace * trace / 4.0 - determinant);
    //double eigenvalue2 = trace / 2.0 - std::sqrt(trace * trace / 4.0 - determinant);

    // 计算主特征向量
    Vector2d direction;
    if (std::abs(cov_xy) > 1e-10) {
      direction.x = eigenvalue1 - cov_yy;
      direction.y = cov_xy;
    } else {
      if (cov_xx > cov_yy) {
        direction.x = 1.0;
        direction.y = 0.0;
      } else {
        direction.x = 0.0;
        direction.y = 1.0;
      }
    }

    // 归一化
    double length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (length > 1e-10) {
      direction.x /= length;
      direction.y /= length;
    }

    return std::atan2(direction.y, direction.x);
  }

  // 统计离群点过滤
  std::vector<Vector2d> statistical_outlier_filter(const std::vector<Vector2d> & points)
  {
    if (points.size() < 4) {return points;}

    int mean_k = stat_mean_k;
    double std_threshold = stat_std_threshold;

    std::vector<double> mean_distances;
    for (const auto & p : points) {
      std::vector<double> distances;
      for (const auto & other : points) {
        if (&p != &other) {
          distances.push_back((p - other).norm());
        }
      }
      std::sort(distances.begin(), distances.end());
      double mean_dist = 0.0;
      int count = std::min(mean_k, static_cast<int>(distances.size()));
      for (int i = 0; i < count; ++i) {
        mean_dist += distances[i];
      }
      mean_dist /= count;
      mean_distances.push_back(mean_dist);
    }

    double mean =
      std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0) / mean_distances.size();
    double std_dev = 0.0;
    for (double d : mean_distances) {
      std_dev += (d - mean) * (d - mean);
    }
    std_dev = sqrt(std_dev / mean_distances.size());

    std::vector<Vector2d> filtered;
    for (size_t i = 0; i < points.size(); ++i) {
      if (mean_distances[i] < mean + std_threshold * std_dev) {
        filtered.push_back(points[i]);
      }
    }
    return filtered;
  }
  // 计算两点之间的欧氏距离
  double distanceTo(const Vector2d & current_points, const Vector2d & other)
  {
    double dx = current_points.x - other.x;
    double dy = current_points.y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  double calculateTranslationWeight(const Vector2d & current_points, const Vector2d & true_points)
  {
    double error = distanceTo(current_points, true_points);
    double weigth_ = std::exp(-sensitivity * error / maxError);
    return weigth_;
  }
  double calculateRotationWeight(const Vector2d & current_points, const Vector2d & true_points)
  {
    double currentAngle = std::atan2(-current_points.y, -current_points.x);
    double realAngle = std::atan2(-true_points.y, -true_points.x);
    double angleDiff = std::abs(realAngle - currentAngle);
    if (angleDiff > M_PI) {
      angleDiff = 2 * M_PI - angleDiff;
    }
    double weigth_ = std::exp(-sensitivity * angleDiff / maxangleError);
    return weigth_;
  }
  // 计算点集的 k 近邻距离
  std::vector<double> compute_knn_distances(const std::vector<Vector2d> & points, int k)
  {
    std::vector<double> avg_distances(points.size(), 0.0);

    for (size_t i = 0; i < points.size(); i++) {
      std::vector<double> distances;

      // 计算当前点到所有其他点的距离
      for (size_t j = 0; j < points.size(); j++) {
        if (i != j) {
          distances.push_back(distanceTo(points[i], points[j]));
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
  double compute_median(std::vector<double> values)
  {
    if (values.empty()) {
      return 0.0;
    }

    std::sort(values.begin(), values.end());
    size_t n = values.size();

    if (n % 2 == 0) {
      return (values[n / 2 - 1] + values[n / 2]) / 2.0;
    } else {
      return values[n / 2];
    }
  }

  // DBSCAN 聚类算法实现
  std::vector<int> dbscan(const std::vector<Vector2d> & points, double eps, int min_samples)
  {
    std::vector<int> labels(points.size(), -1);                 // -1 表示噪声点
    int cluster_id = 0;

    for (size_t i = 0; i < points.size(); i++) {
      if (labels[i] != -1) {
        continue;                         // 已经处理过的点
      }

      // 找到当前点的邻域点
      std::vector<size_t> neighbors;
      for (size_t j = 0; j < points.size(); j++) {
        if (i != j && distanceTo(points[i], points[j]) <= eps) {
          neighbors.push_back(j);
        }
      }

      // 检查是否为核心点
      if (int(neighbors.size()) < min_samples) {
        labels[i] = -1;                         // 标记为噪声点
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
          continue;                             // 已经处理过的点
        }

        labels[current_idx] = cluster_id;

        // 找到当前点的邻域点
        std::vector<size_t> current_neighbors;
        for (size_t j = 0; j < points.size(); j++) {
          if (current_idx != j && distanceTo(points[current_idx], points[j]) <= eps) {
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
  std::vector<int> adaptive_dbscan(const std::vector<Vector2d> & points)
  {
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

  ///// ************************** 圆拟合 **************************** ///
  // 圆拟合结果结构体
  struct CircleFitResult
  {
    Vector2d center;
    double radius;
    double avg_residual;
    double std_residual;
    double max_residual;
    double r_squared;
    bool valid;
  };
  struct Result
  {
    std::vector<double> x;
    double cost;
    bool success;
  };
  /// @brief  使用 Levenberg-Marquardt 算法优化非线性函数; 采用数值优化法计算导数，非Jet或手动法，大范围不准
  /// @param residuals_func  残差函数，输入参数为当前估计值，输出为残差向量
  /// @param initial_guess  初始估计值
  /// @param max_iterations  最大迭代次数
  /// @param tol  收敛 tolerance
  /// @param lambda  阻尼参数，用于处理 ill-conditioned 问题
  /// @return  优化结果，包含优化后的参数和成本
  static Result lm_optimize(
    const std::function<std::vector<double>(const std::vector<double> &)> & residuals_func,
    const std::vector<double> & initial_guess,
    int max_iterations = 100,
    double tol = 1e-6,
    double lambda = 0.01)
  {

    Result result;
    result.x = initial_guess;
    result.success = false;

    int n = initial_guess.size();
    std::vector<double> x = initial_guess;

    // 计算初始残差和成本
    std::vector<double> r = residuals_func(x);
    double cost = 0.0;
    for (double val : r) {cost += val * val;}

    for (int iter = 0; iter < max_iterations; ++iter) {
      // 计算雅可比矩阵的近似（使用有限差分）
      std::vector<std::vector<double>> J(r.size(), std::vector<double>(n, 0.0));
      double h = 1e-6;

      for (int j = 0; j < n; ++j) {
        std::vector<double> x_plus = x;
        x_plus[j] += h;
        std::vector<double> r_plus = residuals_func(x_plus);

        for (int i = 0; i < int(r.size()); ++i) {
          J[i][j] = (r_plus[i] - r[i]) / h;
        }
      }

      // 计算梯度: J^T * r
      std::vector<double> gradient(n, 0.0);
      for (int j = 0; j < n; ++j) {
        for (int i = 0; i < int(r.size()); ++i) {
          gradient[j] += J[i][j] * r[i];
        }
      }

      // 计算Hessian近似: J^T * J
      std::vector<std::vector<double>> H(n, std::vector<double>(n, 0.0));
      for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
          for (int k = 0; k < int(r.size()); ++k) {
            H[i][j] += J[k][i] * J[k][j];
          }
          // 添加阻尼项
          if (i == j) {H[i][j] += lambda;}
        }
      }

      // 解线性系统: H * dx = -gradient
      std::vector<double> dx = solve_linear_system(H, gradient);
      for (int i = 0; i < n; ++i) {dx[i] = -dx[i];}

      // 尝试更新
      std::vector<double> x_new = x;
      for (int i = 0; i < n; ++i) {x_new[i] += dx[i];}

      std::vector<double> r_new = residuals_func(x_new);
      double cost_new = 0.0;
      for (double val : r_new) {cost_new += val * val;}

      // 检查是否接受更新
      if (cost_new < cost) {
        // 接受更新，减小lambda
        x = x_new;
        r = r_new;
        cost = cost_new;
        lambda /= 10.0;
      } else {
        // 拒绝更新，增大lambda
        lambda *= 10.0;
      }

      // 检查收敛
      double grad_norm = 0.0;
      for (double g : gradient) {grad_norm += g * g;}
      grad_norm = std::sqrt(grad_norm);

      if (grad_norm < tol) {
        result.success = true;
        break;
      }
    }

    result.x = x;
    result.cost = cost;
    return result;
  }
  static std::vector<double> solve_linear_system(
    const std::vector<std::vector<double>> & A,
    const std::vector<double> & b)
  {

    int n = A.size();
    std::vector<std::vector<double>> Augmented(n, std::vector<double>(n + 1));

    // 构建增广矩阵
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        Augmented[i][j] = A[i][j];
      }
      Augmented[i][n] = b[i];
    }

    // 前向消元
    for (int i = 0; i < n; ++i) {
      // 寻找主元
      int max_row = i;
      for (int k = i + 1; k < n; ++k) {
        if (std::abs(Augmented[k][i]) > std::abs(Augmented[max_row][i])) {
          max_row = k;
        }
      }

      // 交换行
      std::swap(Augmented[i], Augmented[max_row]);

      // 使主对角元素为1
      double divisor = Augmented[i][i];
      if (std::abs(divisor) < 1e-10) {
        throw std::runtime_error("Matrix is singular or nearly singular");
      }

      for (int j = i; j <= n; ++j) {
        Augmented[i][j] /= divisor;
      }

      // 消元
      for (int k = i + 1; k < n; ++k) {
        double factor = Augmented[k][i];
        for (int j = i; j <= n; ++j) {
          Augmented[k][j] -= factor * Augmented[i][j];
        }
      }
    }

    // 回代
    std::vector<double> x(n);
    for (int i = n - 1; i >= 0; --i) {
      x[i] = Augmented[i][n];
      for (int j = i + 1; j < n; ++j) {
        x[i] -= Augmented[i][j] * x[j];
      }
    }

    return x;
  }
  
  /// @brief  与真实圆拟合，使用最梯度下降法，计算圆的中心和半径，以半径优化
  /// @param points  输入点集， 使用成员变量residual_real 表示实际反光柱半径
  /// @return  圆拟合结果，包含中心和半径
  Vector2d circle_real(const std::vector<Vector2d> & points)
  {
    double radius = residual_real;
    int n = points.size();
    // 计算初始估计值
    double mean_x = 0.0, mean_y = 0.0;
    for (const auto & p : points) {
      mean_x += p.x;
      mean_y += p.y;
    }
    mean_x /= points.size();
    mean_y /= points.size();
    int maxIterations = 1000;
    double learningRate = 0.1;
    double tolerance = 1e-8;
    for (int iter = 0; iter < maxIterations; ++iter) {
      Vector2d test_tmp(0, 0);
      double totalError = 0.0;
      for (const auto & p : points) {
        double dx = p.x - mean_x;
        double dy = p.y - mean_y;
        double d_d = std::sqrt(dx * dx + dy * dy);
        double error = d_d - radius;
        totalError += error * error;
        if (d_d > 1e-10) {
          test_tmp.x += error * dx / d_d;
          test_tmp.y += error * dy / d_d;
        }
      }
      if (totalError < tolerance) {
        break;
      }
      mean_x += learningRate * test_tmp.x / n;
      mean_y += learningRate * test_tmp.y / n;
      if (iter % 100 == 0) {
        learningRate *= 0.9;
      }
    }
    return Vector2d(mean_x, mean_y);
  }

  /// @brief  圆拟合，使用 LM 算法优化非线性函数; 以聚点中心为圆心进行拟合优化
  /// @param points  输入点集
  /// @return  圆拟合结果，包含中心和半径
  CircleFitResult circle_fit(const std::vector<Vector2d> & points)
  {
    CircleFitResult result;
    result.valid = false;
    if (points.size() < 3) {
      return result;
    }

    // 计算初始估计值
    double mean_x = 0.0, mean_y = 0.0;
    for (const auto & p : points) {
      mean_x += p.x;
      mean_y += p.y;
    }
    mean_x /= points.size();
    mean_y /= points.size();

    double mean_r = 0.0;
    for (const auto & p : points) {
      mean_r += std::sqrt(
        (p.x - mean_x) * (p.x - mean_x) +
        (p.y - mean_y) * (p.y - mean_y));
    }
    mean_r /= points.size();

    std::vector<double> initial_guess = {mean_x, mean_y, mean_r};

    // 定义残差函数
    auto residuals_func = [&points](const std::vector<double> & params) -> std::vector<double> {
        double a = params[0];
        double b = params[1];
        double r = params[2];

        std::vector<double> residuals;
        for (const auto & p : points) {
          double residual = (p.x - a) * (p.x - a) + (p.y - b) * (p.y - b) - r * r;
          residuals.push_back(residual);
        }

        return residuals;
      };

    try {
      // 使用自定义的LM算法进行优化
      auto opt_result = lm_optimize(residuals_func, initial_guess);

      if (!opt_result.success) {
        std::cerr << "圆拟合失败: 优化未收敛" << std::endl;
        return result;
      }

      double a = opt_result.x[0];
      double b = opt_result.x[1];
      double r = opt_result.x[2];

      // 计算残差
      std::vector<double> residuals = residuals_func(opt_result.x);
      double sum_residuals_sq = 0.0;
      double max_residual = 0.0;
      double sum_abs_residuals = 0.0;

      for (double residual : residuals) {
        sum_residuals_sq += residual * residual;
        sum_abs_residuals += std::abs(residual);
        if (std::abs(residual) > max_residual) {
          max_residual = std::abs(residual);
        }
      }

      double avg_residual = sum_abs_residuals / residuals.size();

      // 计算残差标准差
      double std_residual = 0.0;
      for (double residual : residuals) {
        std_residual += (residual - avg_residual) * (residual - avg_residual);
      }
      std_residual = std::sqrt(std_residual / residuals.size());

      // 计算R平方
      double ss_total = 0.0;
      double mean_distance = 0.0;

      // 计算到中心的平均距离
      for (const auto & p : points) {
        double distance = std::sqrt((p.x - a) * (p.x - a) + (p.y - b) * (p.y - b));
        mean_distance += distance;
      }
      mean_distance /= points.size();

      // 计算总平方和
      for (const auto & p : points) {
        double distance = std::sqrt((p.x - a) * (p.x - a) + (p.y - b) * (p.y - b));
        ss_total += (distance - mean_distance) * (distance - mean_distance);
      }

      double r_squared = 1.0;
      if (ss_total > 0.0) {
        r_squared = 1.0 - sum_residuals_sq / ss_total;
      }

      // 返回结果
      result.center = Vector2d(a, b);
      result.radius = r;
      result.avg_residual = avg_residual;
      result.std_residual = std_residual;
      result.max_residual = max_residual;
      result.r_squared = r_squared;
      result.valid = true;

    } catch (const std::exception & e) {
      std::cerr << "圆拟合失败: " << e.what() << std::endl;
    }

    return result;
  }

  // 弧度特征计算
  double calculate_arc_feature(const std::vector<Vector2d> & points)
  {
    if (int(points.size()) < arc_min_points) {return 0.0;}

    double primary_dir = get_primary_direction(points);
    //RCLCPP_INFO(this->get_logger(), "info dir: %f", primary_dir);
    Vector2d center;
    center.x = 0;
    center.y = 0;
    for (const auto & p : points) {center += p;}
    center.x = center.x / points.size();
    center.y = center.y / points.size();

    // 旋转点到主方向坐标系

    std::vector<Vector2d> rotated_points;
    for (const auto & p : points) {
      double cos_dir = std::cos(primary_dir);
      double sin_dir = std::sin(primary_dir);
      Vector2d translated = p - center;
      Vector2d rotated;
      rotated.x = cos_dir * translated.x - sin_dir * translated.y;
      rotated.y = sin_dir * translated.x + cos_dir * translated.y;
      rotated_points.push_back(rotated);
    }

    // 按x坐标排序
    std::sort(
      rotated_points.begin(), rotated_points.end(),
      [](const Vector2d & a, const Vector2d & b) {
        return a.x < b.x;
      });

    // 简化实现：计算曲率特征
    std::vector<double> curvatures;
    int window_size = std::min(5, static_cast<int>(points.size()) / 3);

    for (int i = window_size; i < static_cast<int>(points.size()) - window_size; ++i) {
      std::vector<Vector2d> window;
      for (int j = i - window_size; j <= i + window_size; ++j) {
        window.push_back(points[j]);
      }

      CircleFitResult circle = circle_fit(window);
      if (circle.valid && circle.radius > 0) {
        curvatures.push_back(1.0 / circle.radius);
      }
    }

    if (curvatures.size() < 3) {return 0.0;}

    double mean_curvature =
      std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
    double std_curvature = 0.0;
    for (double c : curvatures) {
      std_curvature += (c - mean_curvature) * (c - mean_curvature);
    }
    std_curvature = std::sqrt(std_curvature / curvatures.size());
    //RCLCPP_INFO(this->get_logger(), "info arc: %f,%f", mean_curvature,std_curvature);
    return mean_curvature / (std_curvature + 1e-6);
  }

  // 计算点集的均值
  Vector2d compute_mean(const std::vector<Vector2d> & points)
  {
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto & p : points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    return Vector2d(sum_x / points.size(), sum_y / points.size());
  }
};

} // namespace landmark_localization
