#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <cartographer_ros_msgs/msg/landmark_entry.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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

#include "match_assigner.hpp"

using namespace std::chrono_literals;

//std::ofstream outFile("./test_cpp_circle_r.txt");

class ReflectorDetectorCircle : public rclcpp::Node
{
public:
  ReflectorDetectorCircle()
  : Node("reflector_detector_maker"), detection_id_counter_(0)
  {
    // 参数声明
    this->declare_parameter("intensity_threshold_use", 200);
    this->declare_parameter("cluster_eps", 0.04);
    this->declare_parameter("min_cluster_points", 10);
    this->declare_parameter("diameter_min", 0.05);
    this->declare_parameter("diameter_max", 0.12);
    this->declare_parameter("residual_avg_threshold", 0.01);
    this->declare_parameter("residual_std_threshold", 0.005);
    this->declare_parameter("residual_max_threshold", 0.02);
    this->declare_parameter("stat_mean_k", 5);
    this->declare_parameter("stat_std_threshold", 1.0);
    this->declare_parameter("max_history_age", 3);
    this->declare_parameter("match_distance_threshold", 0.1);
    this->declare_parameter("arc_threshold", 0.1);
    this->declare_parameter("max_arc_feature", 30.0);
    this->declare_parameter("arc_min_points", 8);
    this->declare_parameter("direction_tolerance", 0.785);
    this->declare_parameter("residual_real", 0.032);
    this->declare_parameter("sensitivity", 2.0);
    this->declare_parameter("maxError", 1.0);
    this->declare_parameter("maxangleError", 0.1);

    // 订阅和发布
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ReflectorDetectorCircle::scan_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
      "/landmark", 10);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/reflector_markers", 10);

    RCLCPP_INFO(this->get_logger(), "反光柱检测节点初始化完成");
  }

  void add_match_assigner(const std::shared_ptr<MatchAssigner> & match_assigner)
  {
    match_assigner_ = match_assigner;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  std::shared_ptr<MatchAssigner> match_assigner_;
  std::map<int, HistoryEntry> detection_history_;
  int detection_id_counter_;
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
  Vector2d get_final_coor(Vector2d points, double angle)
  {
    double final_x = -std::cos(angle) * points.x - std::sin(angle) * points.y;
    double final_y = std::sin(angle) * points.x - std::cos(angle) * points.y;
    return Vector2d(final_x, final_y);
  }

  // 统计离群点过滤
  std::vector<Vector2d> statistical_outlier_filter(const std::vector<Vector2d> & points)
  {
    if (points.size() < 10) {return points;}

    int mean_k = this->get_parameter("stat_mean_k").as_int();
    double std_threshold = this->get_parameter("stat_std_threshold").as_double();

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
    double sensitivity = this->get_parameter("sensitivity").as_double();
    double maxError = this->get_parameter("maxError").as_double();
    double error = distanceTo(current_points, true_points);
    double weigth_ = std::exp(-sensitivity * error / maxError);
    return weigth_;
  }
  double calculateRotationWeight(const Vector2d & current_points, const Vector2d & true_points)
  {
    double sensitivity = this->get_parameter("sensitivity").as_double();
    double maxangleError = this->get_parameter("maxangleError").as_double();
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
    std::vector<int> labels(points.size(), -1);             // -1 表示噪声点
    int cluster_id = 0;

    for (size_t i = 0; i < points.size(); i++) {
      if (labels[i] != -1) {
        continue;                 // 已经处理过的点
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
        labels[i] = -1;                 // 标记为噪声点
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
          continue;                   // 已经处理过的点
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
    int min_cluster_points = this->get_parameter("min_cluster_points").as_int();
    double cluster_eps = this->get_parameter("cluster_eps").as_double();
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
  Vector2d circle_real(const std::vector<Vector2d> & points)
  {
    double radius = this->get_parameter("residual_real").as_double();
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
    int arc_min_points = this->get_parameter("arc_min_points").as_int();
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

  // 判断方向是否朝向雷达
  bool is_direction_towards_radar(
    double direction_theta, double center_x, double center_y,
    double tolerance = 0.785)
  {
    double towards_radar_theta = std::atan2(-center_y, -center_x);
    double angle_diff = std::abs(direction_theta - towards_radar_theta);
    angle_diff = std::min(angle_diff, 2 * M_PI - angle_diff);
    return angle_diff < tolerance;
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
  // 四舍五入到指定小数位
  double roundTo(double value, int decimals)
  {
    double factor = std::pow(10.0, decimals);
    return std::round(value * factor) / factor;
  }
  using DetectionHash = std::tuple<double, double, double, double, double, double, double, double>;
  // 创建检测哈希
  DetectionHash createDetectionHash(const Detection & detection)
  {
    const auto & pos = detection.pose.pose.position;
    const auto & ori = detection.pose.pose.orientation;

    return std::make_tuple(
      roundTo(pos.x, 4),
      roundTo(pos.y, 4),
      roundTo(pos.z, 4),
      roundTo(ori.x, 4),
      roundTo(ori.y, 4),
      roundTo(ori.z, 4),
      roundTo(ori.w, 4),
      roundTo(detection.diameter, 4)
    );
  }
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
  // 匹配当前检测与历史记录
  std::map<DetectionHash, std::pair<int, HistoryEntry>> matchCurrentToHistory(
    const std::vector<Detection> & current_detections)
  {
    double match_distance_threshold_ = this->get_parameter("match_distance_threshold").as_double();
    std::map<DetectionHash, std::pair<int, HistoryEntry>> matches;

    // 初始化匹配表
    for (const auto & detection : current_detections) {
      matches[createDetectionHash(detection)] = std::make_pair(0, HistoryEntry());
    }

    // 创建历史记录的哈希映射
    std::map<DetectionHash, std::pair<int, HistoryEntry>> hist_hashes;
    for (const auto & [id, entry] : detection_history_) {
      Detection dummy_detection{entry.pose, entry.diameter, entry.confidence, entry.translationW,
        entry.rotationW};
      hist_hashes[createDetectionHash(dummy_detection)] = std::make_pair(id, entry);
    }

    // 匹配当前检测与历史记录
    for (const auto & detection : current_detections) {
      auto current_hash = createDetectionHash(detection);
      Point current_pos;
      current_pos.x = detection.pose.pose.position.x;
      current_pos.y = detection.pose.pose.position.y;
      current_pos.z = detection.pose.pose.position.z;
      std::pair<int, HistoryEntry> best_match;
      double min_distance = std::numeric_limits<double>::max();

      for (const auto & [hist_hash, hist_info] : hist_hashes) {
        Point hist_pos;
        hist_pos.x = hist_info.second.pose.pose.position.x;
        hist_pos.y = hist_info.second.pose.pose.position.y;
        hist_pos.z = hist_info.second.pose.pose.position.z;
        double distance = current_pos.distanceTo(hist_pos);

        if (distance < min_distance && distance < match_distance_threshold_) {
          min_distance = distance;
          best_match = hist_info;
        }
      }

      if (min_distance < match_distance_threshold_) {
        matches[current_hash] = best_match;
      }
    }

    return matches;
  }

  struct Quaternion
  {
    double x, y, z, w;

    Quaternion()
    : x(0), y(0), z(0), w(1) {}
    Quaternion(double x, double y, double z, double w)
    : x(x), y(y), z(z), w(w) {}

    // 四元数点积
    double dot(const Quaternion & other) const
    {
      return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    // 四元数归一化
    Quaternion normalized() const
    {
      double norm = std::sqrt(x * x + y * y + z * z + w * w);
      if (norm > 0) {
        return Quaternion(x / norm, y / norm, z / norm, w / norm);
      }
      return *this;
    }

    // 四元数标量乘法
    Quaternion operator*(double scalar) const
    {
      return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    // 四元数加法
    Quaternion operator+(const Quaternion & other) const
    {
      return Quaternion(x + other.x, y + other.y, z + other.z, w + other.w);
    }
  };

  // 融合两个位姿
  geometry_msgs::msg::PoseStamped fusePoses(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    double weight1, double weight2)
  {
    geometry_msgs::msg::PoseStamped fused_pose;
    double total_weight = weight1 + weight2;

    // 融合位置
    fused_pose.pose.position.x =
      (pose1.pose.position.x * weight1 + pose2.pose.position.x * weight2) / total_weight;
    fused_pose.pose.position.y =
      (pose1.pose.position.y * weight1 + pose2.pose.position.y * weight2) / total_weight;
    fused_pose.pose.position.z = 0.0;     // 假设z坐标为0

    // 融合方向（四元数）
    Quaternion q1;
    q1.x = pose1.pose.orientation.x;
    q1.y = pose1.pose.orientation.y;
    q1.z = pose1.pose.orientation.z;
    q1.w = pose1.pose.orientation.w;
    Quaternion q2;
    q2.x = pose2.pose.orientation.x;
    q2.y = pose2.pose.orientation.y;
    q2.z = pose2.pose.orientation.z;
    q2.w = pose2.pose.orientation.w;
    // 确保四元数在同一半球
    if (q1.dot(q2) < 0) {
      q2 = Quaternion(-q2.x, -q2.y, -q2.z, -q2.w);
    }

    // 球面线性插值
    double t = weight1 / total_weight;
    Quaternion q_fused = (q1 * t + q2 * (1.0 - t)).normalized();
    fused_pose.pose.orientation.x = q_fused.x;
    fused_pose.pose.orientation.y = q_fused.y;
    fused_pose.pose.orientation.z = q_fused.z;
    fused_pose.pose.orientation.w = q_fused.w;

    return fused_pose;
  }
  std::vector<Detection> updateTemporalFilter(const std::vector<Detection> & current_detections)
  {
    auto matches = matchCurrentToHistory(current_detections);
    int max_history_age_ = this->get_parameter("max_history_age").as_int();
    // 创建检测哈希到检测对象的映射
    std::map<DetectionHash, Detection> detection_hash_map;
    for (const auto & det : current_detections) {
      detection_hash_map[createDetectionHash(det)] = det;
    }

    std::vector<Detection> filtered_detections;

    for (const auto & [detection_hash, match_info] : matches) {
      const auto & detection = detection_hash_map[detection_hash];

      if (match_info.first == 0) {       // 未匹配到历史记录
        // 新检测结果，添加到历史
        detection_history_[detection_id_counter_] = {
          detection.pose,
          detection.diameter,
          detection.confidence,
          detection.translationW,
          detection.rotationW,
          0
        };
        filtered_detections.push_back(detection);
        detection_id_counter_++;
      } else {
        // 匹配到历史记录，融合结果
        int hist_id = match_info.first;
        const auto & hist_data = match_info.second;

        geometry_msgs::msg::PoseStamped fused_pose = fusePoses(
          detection.pose,
          hist_data.pose,
          detection.confidence,
          hist_data.confidence
        );

        double fused_confidence = (detection.confidence + hist_data.confidence * 0.8) / 1.8;

        detection_history_[hist_id] = {
          fused_pose,
          detection.diameter,
          fused_confidence,
          detection.translationW,
          detection.rotationW,
          0           // 重置年龄
        };

        filtered_detections.push_back(
        {
          fused_pose,
          detection.diameter,
          fused_confidence,
          detection.translationW,
          detection.rotationW
        });
      }
    }

    // 老化未匹配的历史记录
    for (auto it = detection_history_.begin(); it != detection_history_.end(); ) {
      auto & [id, entry] = *it;

      if (entry.age > max_history_age_) {
        it = detection_history_.erase(it);
      } else {
        entry.confidence *= 0.95;
        entry.age += 1;
        ++it;
      }
    }

    return filtered_detections;
  }

  visualization_msgs::msg::Marker CreateLandmarkMarker(
    int landmark_index,
    const geometry_msgs::msg::PoseStamped & landmark_pose,
    const std::string & frame_id,
    rclcpp::Time scan_time)
  {
    double kLandmarkMarkerScale = 0.1;
    visualization_msgs::msg::Marker marker;
    marker.ns = "reflective_circles";
    marker.id = landmark_index;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.header.stamp = scan_time;
    marker.header.frame_id = frame_id;
    marker.scale.x = kLandmarkMarkerScale;
    marker.scale.y = kLandmarkMarkerScale;
    marker.scale.z = kLandmarkMarkerScale;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose = landmark_pose.pose;
    return marker;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int intensity_thresh = this->get_parameter("intensity_threshold_use").as_int();
    //RCLCPP_INFO(this->get_logger(), "实际使用的强度阈值: %d", intensity_thresh);

    // 提取高强度点
    std::vector<Vector2d> points;
    std::vector<float> high_intensities;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (msg->intensities[i] > intensity_thresh &&
        msg->ranges[i] > msg->range_min &&
        msg->ranges[i] < msg->range_max &&
        !std::isnan(msg->ranges[i]))
      {

        double angle = msg->angle_min + i * msg->angle_increment;
        Vector2d tmp_p;
        tmp_p.x = msg->ranges[i] * std::cos(angle);
        tmp_p.y = msg->ranges[i] * std::sin(angle);
        points.push_back(tmp_p);
        high_intensities.push_back(msg->intensities[i]);
      }
    }

    //RCLCPP_INFO(this->get_logger(), "通过强度筛选的点数量: %zu", points.size());
    if (points.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "未检测到高强度点，不发布结果");
      return;
    }

    // 去噪
    if (points.size() > 10) {
      points = statistical_outlier_filter(points);
    }

    // 聚类
    std::vector<int> labels;
    if (int(points.size()) >= this->get_parameter("min_cluster_points").as_int()) {
      labels = adaptive_dbscan(points);
      //RCLCPP_INFO(this->get_logger(), "info lable size: %d", int(labels.size()));
    } else {
      labels = std::vector<int>(points.size(), -1);
    }

    // 提取有效聚类点
    std::vector<Vector2d> valid_points;
    std::vector<int> valid_labels;
    for (size_t i = 0; i < points.size(); ++i) {
      if (labels[i] != -1) {
        valid_points.push_back(points[i]);
        valid_labels.push_back(labels[i]);
      }
    }

    if (valid_points.empty()) {
      RCLCPP_INFO(this->get_logger(), "未检测到有效聚类，不发布结果");
      return;
    }
    std::vector<int> unique_labels = valid_labels;
    std::sort(unique_labels.begin(), unique_labels.end());
    unique_labels.erase(
      std::unique(unique_labels.begin(), unique_labels.end()),
      unique_labels.end());


    double diameter_min = this->get_parameter("diameter_min").as_double();
    double diameter_max = this->get_parameter("diameter_max").as_double();
    cartographer_ros_msgs::msg::LandmarkList landamrk_list;
    std::vector<cartographer_ros_msgs::msg::LandmarkEntry> poses_array;
    landamrk_list.header = msg->header;
    std::vector<Detection> current_detections;
    for (int label : unique_labels) {
      std::vector<Vector2d> cluster;
      for (size_t i = 0; i < valid_labels.size(); ++i) {
        if (valid_labels[i] == label) {
          cluster.push_back(valid_points[i]);
        }
      }
      RCLCPP_INFO(this->get_logger(), "info dbscan lable: %d", int(cluster.size()));
      CircleFitResult circle = circle_fit(cluster);
      RCLCPP_INFO(this->get_logger(), "info arc_feature: %f,%f,%f",circle.center.x,circle.center.y,circle.radius*2);
      if (circle.valid && circle.r_squared >= 0.85) {
        double diameter = 2 * circle.radius;
        if (diameter > diameter_min && diameter < diameter_max) {
          double arc_feature = calculate_arc_feature(cluster);
          double arc_threshold = this->get_parameter("arc_threshold").as_double();
          double max_arc = this->get_parameter("max_arc_feature").as_double();
          //RCLCPP_INFO(this->get_logger(), "info arc_feature: %f",arc_feature);
          if (circle.avg_residual < this->get_parameter("residual_avg_threshold").as_double() &&
            circle.std_residual < this->get_parameter("residual_std_threshold").as_double() &&
            circle.max_residual < this->get_parameter("residual_max_threshold").as_double() &&
            arc_feature > arc_threshold && arc_feature < max_arc)
          {
            double x_ = -circle.center.x;
            double y_ = -circle.center.y;
            double normal_theta = std::atan2(y_, x_);
            if (normal_theta > M_PI) {
              normal_theta = 2 * M_PI - normal_theta;
            }
            Vector2d real_p = circle_real(cluster);
            double t_w = calculateTranslationWeight(circle.center, real_p);
            double r_w = calculateRotationWeight(circle.center, real_p);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg->header;
            pose.pose.position.x = circle.center.x;
            pose.pose.position.y = circle.center.y;
            pose.pose.orientation.z = std::sin(normal_theta / 2);
            pose.pose.orientation.w = std::cos(normal_theta / 2);

            // 计算置信度
            double confidence = std::min(1.0, 0.8 + 0.2 * (points.size() / 20.0)) *
              std::min(1.0, circle.r_squared) *
              std::min(1.0, arc_feature / arc_threshold);


            Detection tmp_results;
            tmp_results.pose = pose;
            tmp_results.diameter = diameter;
            tmp_results.confidence = confidence;
            tmp_results.translationW = t_w;
            tmp_results.rotationW = r_w;
            current_detections.push_back(tmp_results);
          }
        }
        RCLCPP_INFO(this->get_logger(), "完成反光柱聚类检测");
      }
    }
    // 进行检测结果发布
    // 发布结果（简化时间滤波）
    match_assigner_->setLaserFrame(msg->header.frame_id);
    if (!current_detections.empty() && match_assigner_->isLandmarkDetectorOK()) {
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "检测到反光柱,进行匹配跟踪....");
      // 选择置信度最高的检测结果
      cartographer_ros_msgs::msg::LandmarkEntry poseSimple;
      std::vector<ReflectorBar> local_detections = match_assigner_->assignLandmarkToReflectorBar(
        current_detections);
      // 发布检测结果使用VisualMarker
      visualization_msgs::msg::MarkerArray marker_array;
      for (auto & landmark : local_detections) {
        poseSimple.tracking_from_landmark_transform = landmark.g_detection_.pose.pose;
        poseSimple.translation_weight = landmark.g_detection_.translationW;
        poseSimple.rotation_weight = landmark.g_detection_.rotationW;
        poseSimple.id = landmark.id_str_;
        poses_array.push_back(poseSimple);
        // 发布可视化标记
        marker_array.markers.push_back(
          CreateLandmarkMarker(
            landmark.id_, landmark.g_detection_.pose,
            msg->header.frame_id, msg->header.stamp));
      }
      // 发布reflectors可视化标记
      marker_publisher_->publish(marker_array);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "检测出反光柱: " << local_detections.size() << "个");
      for(auto & landmark : local_detections) {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "id: " << landmark.id_str_ << 
          ", 圆心(" <<landmark.g_detection_.pose.pose.position.x <<
          "," << landmark.g_detection_.pose.pose.position.y << ") tw="<<
          landmark.g_detection_.translationW << " rw=" <<
          landmark.g_detection_.rotationW);
      }
    } else {
      RCLCPP_DEBUG(this->get_logger(), "未检测到有效反光柱，不发布结果");
    }

    landamrk_list.landmarks = poses_array;
    publisher_->publish(landamrk_list);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReflectorDetectorCircle>();
  auto match_assigner_ = std::make_shared<MatchAssigner>(node);
  node->add_match_assigner(match_assigner_);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
