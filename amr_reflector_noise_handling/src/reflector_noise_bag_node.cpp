#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

#include "amr_reflector_noise_handling/circle_fitter.hpp"
#include "amr_reflector_noise_handling/fixed_dbscan.hpp"
#include "amr_reflector_noise_handling/fractal_dimension.hpp"
#include "amr_reflector_noise_handling/geometric_validator.hpp"
#include "amr_reflector_noise_handling/improved_interpolation.hpp"
#include "amr_reflector_noise_handling/pca_shape_classifier.hpp"
#include "amr_reflector_noise_handling/practical_descriptor.hpp"
#include "amr_reflector_noise_handling/reflector_tracker.hpp"
#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/msg_conversion.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"
#include <termios.h> // 终端控制头文件
#include <unistd.h>  // STDIN_FILENO

using namespace amr_reflector_noise_handling;

// 关闭终端行缓冲和回显，实现无回车读单个字符
char get_char_without_enter() {
  struct termios old_attr, new_attr;
  tcgetattr(STDIN_FILENO, &old_attr); // 获取原有终端属性
  new_attr = old_attr;
  new_attr.c_lflag &= ~(ICANON | ECHO); // 关闭行缓冲(ICANON)、关闭回显(ECHO)
  tcsetattr(STDIN_FILENO, TCSANOW, &new_attr); // 立即应用新属性

  char c = getchar(); // 此时无需回车，输入单个字符立即返回

  tcsetattr(STDIN_FILENO, TCSANOW, &old_attr); // 恢复原有终端属性（必做！）
  return c;
}

// 位姿点结构体（带时间戳）
struct PosePoint {
  double timestamp;         // 时间戳（秒）
  transforms::Rigid3d pose; // 位姿（x,y,z,quat）

  // 按时间戳排序
  bool operator<(const PosePoint &other) const {
    return timestamp < other.timestamp;
  }
};

// 三次样条类（单维度，支持多边界条件）
class CubicSpline1D {
public:
  // 边界条件类型
  enum class BoundaryType {
    NATURAL, // 自然样条：两端二阶导数=0
    CLAMPED, // 夹紧样条：指定两端一阶导数（速度），两个端点的速度
    PERIODIC // 周期样条：两端一阶/二阶导数连续（闭环轨迹）
  };

  // 构造函数：输入时间戳、采样值、边界条件
  CubicSpline1D(const std::vector<double> &ts, const std::vector<double> &ys,
                BoundaryType bc_type = BoundaryType::NATURAL,
                double dy0 = 0.0, // 夹紧样条：t0处的一阶导数（速度）
                double dyn = 0.0) // 夹紧样条：tn处的一阶导数（速度）
  {
    if (ts.size() != ys.size() || ts.size() < 2) {
      throw std::runtime_error(
          "采样点数量不足（至少2个）或时间戳/值长度不匹配");
    }

    // 保存时间戳和采样值（已排序）
    ts_ = ts;
    ys_ = ys;
    n_ = ts.size() - 1; // 分段数 = 采样点数 - 1

    // 计算相邻时间戳的间隔 h
    hs_.resize(n_);
    for (int i = 0; i < n_; ++i) {
      hs_[i] = ts_[i + 1] - ts_[i];
      if (hs_[i] <= 1e-6) {
        throw std::runtime_error("时间戳重复或间隔过小");
      }
    }

    // 求解二阶导数（m）：核心三对角方程组
    solve_second_derivatives(bc_type, dy0, dyn);

    // 预计算分段多项式的系数（a,b,c,d）：S_i(t) = a_i + b_i*(t-ti) +
    // c_i*(t-ti)^2 + d_i*(t-ti)^3
    compute_polynomial_coefficients();
  }

  // 插值任意时间戳的数值
  double interpolate(double t) const {
    // 找到t所在的分段区间
    int idx = find_segment_index(t);
    if (idx < 0)
      return ys_.front(); // t < t0，返回第一个值
    if (idx >= n_)
      return ys_.back(); // t > tn，返回最后一个值

    double dt = t - ts_[idx];
    // 分段三次多项式：S_i(t) = a + b*dt + c*dt² + d*dt³
    const auto &coeff = coeffs_[idx];
    return coeff.a + coeff.b * dt + coeff.c * dt * dt + coeff.d * dt * dt * dt;
  }

  // 插值一阶导数（速度）
  double derivative1(double t) const {
    int idx = find_segment_index(t);
    if (idx < 0 || idx >= n_)
      return 0.0;

    double dt = t - ts_[idx];
    const auto &coeff = coeffs_[idx];
    return coeff.b + 2 * coeff.c * dt + 3 * coeff.d * dt * dt;
  }

  // 插值二阶导数（加速度）
  double derivative2(double t) const {
    int idx = find_segment_index(t);
    if (idx < 0 || idx >= n_)
      return 0.0;

    const auto &coeff = coeffs_[idx];
    return 2 * coeff.c + 6 * coeff.d * (t - ts_[idx]);
  }

private:
  // 分段多项式系数
  struct Coeff {
    double a, b, c, d;
  };

  std::vector<double> ts_;    // 时间戳
  std::vector<double> ys_;    // 采样值
  int n_;                     // 分段数
  std::vector<double> hs_;    // 相邻时间戳间隔 h_i = t_{i+1} - t_i
  std::vector<double> ms_;    // 二阶导数 m_i = S''(t_i)
  std::vector<Coeff> coeffs_; // 分段多项式系数

  // 找到t所在的分段索引
  int find_segment_index(double t) const {
    if (t <= ts_[0])
      return -1;
    if (t >= ts_.back())
      return n_;

    // 二分查找（高效）
    int left = 0, right = n_;
    while (left < right) {
      int mid = (left + right) / 2;
      if (ts_[mid + 1] > t) {
        right = mid;
      } else {
        left = mid + 1;
      }
    }
    return left;
  }

  // 求解二阶导数m（核心：三对角方程组）
  void solve_second_derivatives(BoundaryType bc_type, double dy0, double dyn) {
    ms_.resize(ts_.size(), 0.0);
    std::vector<double> a(n_, 0.0), b(n_ + 1, 0.0), c(n_, 0.0), d(n_ + 1, 0.0);

    // 构建三对角方程组：A*m = d
    for (int i = 1; i < n_; ++i) {
      a[i - 1] = hs_[i - 1];
      b[i] = 2 * (hs_[i - 1] + hs_[i]);
      c[i] = hs_[i];
      d[i] = 6 * ((ys_[i + 1] - ys_[i]) / hs_[i] -
                  (ys_[i] - ys_[i - 1]) / hs_[i - 1]);
    }

    // 应用边界条件
    switch (bc_type) {
    case BoundaryType::NATURAL: {
      // 自然样条：m0=0, mn=0
      b[0] = 1.0;
      c[0] = 0.0;
      d[0] = 0.0;
      b[n_] = 1.0;
      a[n_ - 1] = 0.0;
      d[n_] = 0.0;
      break;
    }
    case BoundaryType::CLAMPED: {
      // 夹紧样条：指定m0和mn的约束（由一阶导数推导）
      b[0] = 2 * hs_[0];
      c[0] = hs_[0];
      d[0] = 6 * ((ys_[1] - ys_[0]) / hs_[0] - dy0);

      b[n_] = 2 * hs_[n_ - 1];
      a[n_ - 1] = hs_[n_ - 1];
      d[n_] = 6 * (dyn - (ys_[n_] - ys_[n_ - 1]) / hs_[n_ - 1]);
      break;
    }
    case BoundaryType::PERIODIC: {
      // 周期样条：m0=mn，S'(t0)=S'(tn)，S''(t0)=S''(tn)
      // 重构方程组（简化版，仅适配周期场景）
      b[0] = 2 * (hs_[0] + hs_[n_ - 1]);
      c[0] = hs_[0];
      a[0] = hs_[n_ - 1];
      d[0] = 6 * ((ys_[1] - ys_[0]) / hs_[0] -
                  (ys_[n_] - ys_[n_ - 1]) / hs_[n_ - 1]);

      for (int i = 1; i < n_; ++i) {
        a[i] = hs_[i - 1];
        b[i] = 2 * (hs_[i - 1] + hs_[i]);
        c[i] = hs_[i];
        d[i] = 6 * ((ys_[i + 1] - ys_[i]) / hs_[i] -
                    (ys_[i] - ys_[i - 1]) / hs_[i - 1]);
      }
      // 手动设置m0=mn
      ms_[0] = ms_[n_];
      break;
    }
    }

    // 托马斯算法（追赶法）求解三对角方程组
    thomas_algorithm(a, b, c, d, ms_);
  }

  // 托马斯算法：求解三对角方程组 A*x = b
  void thomas_algorithm(const std::vector<double> &a,
                        const std::vector<double> &b,
                        const std::vector<double> &c,
                        const std::vector<double> &d, std::vector<double> &x) {
    int n = x.size();
    std::vector<double> c_prime(n, 0.0), d_prime(n, 0.0);

    // 前向消去
    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];
    for (int i = 1; i < n; ++i) {
      double temp = b[i] - a[i - 1] * c_prime[i - 1];
      c_prime[i] = c[i] / temp;
      d_prime[i] = (d[i] - a[i - 1] * d_prime[i - 1]) / temp;
    }

    // 反向回代
    x[n - 1] = d_prime[n - 1];
    for (int i = n - 2; i >= 0; --i) {
      x[i] = d_prime[i] - c_prime[i] * x[i + 1];
    }

    // 周期样条补充：m0=mn
    // if (bc_type == BoundaryType::PERIODIC) {
    //   x[0] = x[n - 1];
    // }
  }

  // 计算分段多项式系数
  void compute_polynomial_coefficients() {
    coeffs_.resize(n_);
    for (int i = 0; i < n_; ++i) {
      double h = hs_[i];
      coeffs_[i].a = ys_[i];
      coeffs_[i].b =
          (ys_[i + 1] - ys_[i]) / h - h * (ms_[i + 1] + 2 * ms_[i]) / 6;
      coeffs_[i].c = ms_[i] / 2;
      coeffs_[i].d = (ms_[i + 1] - ms_[i]) / (6 * h);
    }
  }
};

class PoseCubicSpline {
public:
  // 构造函数：输入位姿采样点 + 位置边界条件
  PoseCubicSpline(const std::vector<PosePoint> &pose_points,
                  CubicSpline1D::BoundaryType pos_bc_type =
                      CubicSpline1D::BoundaryType::NATURAL,
                  double vx0 = 0.0,
                  double vxn = 0.0, // x轴初始/结束速度（夹紧样条用）
                  double vy0 = 0.0, double vyn = 0.0, // y轴初始/结束速度
                  double vz0 = 0.0, double vzn = 0.0) // z轴初始/结束速度
  {
    // 1. 预处理：时间戳排序+去重
    std::vector<PosePoint> sorted_poses = pose_points;
    std::sort(sorted_poses.begin(), sorted_poses.end());
    auto last = std::unique(sorted_poses.begin(), sorted_poses.end(),
                            [](const PosePoint &a, const PosePoint &b) {
                              return fabs(a.timestamp - b.timestamp) < 1e-6;
                            });
    sorted_poses.erase(last, sorted_poses.end());

    if (sorted_poses.size() < 2) {
      throw std::runtime_error("位姿采样点数量不足（至少2个）");
    }

    // 2. 提取各维度数据
    std::vector<double> ts, xs, ys, zs;
    std::vector<Eigen::Quaterniond> quats;
    for (const auto &p : sorted_poses) {
      ts.push_back(p.timestamp);
      xs.push_back(p.pose.translation().x());
      ys.push_back(p.pose.translation().y());
      zs.push_back(p.pose.translation().z());
      quats.push_back(p.pose.rotation().normalized());
    }

    // 3. 位置拟合（x/y/z分别用三次样条）
    spline_x_ = std::make_unique<CubicSpline1D>(ts, xs, pos_bc_type, vx0, vxn);
    spline_y_ = std::make_unique<CubicSpline1D>(ts, ys, pos_bc_type, vy0, vyn);
    spline_z_ = std::make_unique<CubicSpline1D>(ts, zs, pos_bc_type, vz0, vzn);

    // 4. 姿态拟合：偏航角用周期三次样条，四元数用SQUAD
    // 保存四元数和时间戳（SQUAD插值用）
    ts_ = ts;
    quats_ = quats;
  }

  // 插值任意时间戳的位姿
  PosePoint interpolate(double t) const {
    PosePoint res;
    res.timestamp = t;

    // 1. 位置插值（三次样条）
    double x = spline_x_->interpolate(t);
    double y = spline_y_->interpolate(t);
    double z = spline_z_->interpolate(t);

    // 2. 四元数插值（SQUAD，C1连续）
    Eigen::Quaterniond quat = interpolate_quaternion_squad(t);
    res.pose = transforms::Rigid3d(Eigen::Vector3d(x, y, z), quat);
    return res;
  }

private:
  // 位置三次样条
  std::unique_ptr<CubicSpline1D> spline_x_;
  std::unique_ptr<CubicSpline1D> spline_y_;
  std::unique_ptr<CubicSpline1D> spline_z_;

  // 四元数插值相关
  std::vector<double> ts_;
  std::vector<Eigen::Quaterniond> quats_;

  // SQUAD插值四元数（C1连续）
  Eigen::Quaterniond interpolate_quaternion_squad(double t) const {
    // 找到t所在区间
    int idx = find_segment_index(t);
    if (idx < 0)
      return quats_.front();
    if (idx >= (int)ts_.size() - 1)
      return quats_.back();

    int i0 = idx;
    int i1 = idx + 1;
    int i_prev = (i0 == 0) ? ts_.size() - 2 : i0 - 1; // 前一个点（周期处理）
    int i_next = (i1 == (int)ts_.size() - 1) ? 1 : i1 + 1; // 后一个点

    double t0 = ts_[i0];
    double t1 = ts_[i1];
    double s = (t - t0) / (t1 - t0);

    // 计算SQUAD中间控制点
    Eigen::Quaterniond a =
        squad_intermediate(quats_[i_prev], quats_[i0], quats_[i1]);
    Eigen::Quaterniond b =
        squad_intermediate(quats_[i0], quats_[i1], quats_[i_next]);

    // SQUAD插值
    return squad(quats_[i0], quats_[i1], a, b, s);
  }

  // SQUAD中间控制点计算
  Eigen::Quaterniond
  squad_intermediate(const Eigen::Quaterniond &q_prev,
                     const Eigen::Quaterniond &q_curr,
                     const Eigen::Quaterniond &q_next) const {
    Eigen::Quaterniond q_inv = q_curr.inverse();
    return q_curr * ((q_inv * q_next).slerp(0.5, q_inv * q_prev)).normalized();
  }

  // SQUAD核心插值
  Eigen::Quaterniond squad(const Eigen::Quaterniond &q0,
                           const Eigen::Quaterniond &q1,
                           const Eigen::Quaterniond &a,
                           const Eigen::Quaterniond &b, double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double s = 2 * t3 - 3 * t2 + 1;
    double v = t3 - 2 * t2 + t;
    double w = t3 - t2;
    double u = -2 * t3 + 3 * t2;

    Eigen::Quaterniond q_slerp1 = q0.slerp(t, q1);
    Eigen::Quaterniond q_slerp2 = a.slerp(t, b);
    return q_slerp1.slerp(2 * t * (1 - t), q_slerp2).normalized();
  }

  // 找时间戳分段索引
  int find_segment_index(double t) const {
    if (t <= ts_[0])
      return -1;
    if (t >= ts_.back())
      return (int)ts_.size() - 1;

    int left = 0, right = (int)ts_.size() - 1;
    while (left < right) {
      int mid = (left + right) / 2;
      if (ts_[mid + 1] > t) {
        right = mid;
      } else {
        left = mid + 1;
      }
    }
    return left;
  }
};

/**
 * @brief Frame data structure for storing scan and odometry information
 */
struct FrameData {
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  int64_t timestamp; // nanoseconds
  std::vector<int64_t> between_next_odoms;
  size_t odom_count;
  size_t frame_index; // laserscan的索引

  // Compensated point cloud (after distortion correction)
  std::vector<Point> compensated_points;
  std::vector<Point> filtered_points;

  // Detected reflectors
  std::vector<DetectedReflector> reflectors;

  // Global pose in world frame
  transforms::Rigid3d global_pose;

  FrameData() : frame_index(0) {}
};

/**
 * @brief Global pose tracking using odometry
 */
class PoseTracker {
public:
  PoseTracker() : initialized_(false) {}

  /**
   * @brief Update global pose using odometry
   */
  void
  update(std::shared_ptr<FrameData> &laser_frame,
         const std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr>
             &odom_queue) {
    // TODO: 使用OdometryQueue进行LaserScan数据的CSplines拟合
    // 2. 使用CSpline进行插值求取laser帧各个扫描点的全局位姿；构建filtered点云
    if (!initialized_) {
      // Initialize with first odometry
      initialized_ = true;
      return;
    }
  }

  /**
   * @brief Get current global pose
   */
  geometry_msgs::msg::Pose getGlobalPose() const { return global_pose_; }

  /**
   * @brief Reset tracker
   */
  void reset() {
    initialized_ = false;
    global_pose_ = geometry_msgs::msg::Pose();
    initial_pose_ = geometry_msgs::msg::Pose();
    prev_odom_ = nullptr;
  }

  /**
   * @brief Get trajectory points
   */
  const std::vector<geometry_msgs::msg::PoseStamped> &getTrajectory() const {
    return trajectory_;
  }

  /**
   * @brief Add trajectory point
   */
  void addTrajectoryPoint(const geometry_msgs::msg::Pose &pose,
                          const rclcpp::Time &time) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = "odom";
    trajectory_.push_back(pose_stamped);
  }

private:
  bool initialized_;
  geometry_msgs::msg::Pose global_pose_;
  geometry_msgs::msg::Pose initial_pose_;
  nav_msgs::msg::Odometry::SharedPtr prev_odom_;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory_;

  /**
   * @brief Compute relative transform between two odometry poses
   */
  geometry_msgs::msg::Transform
  computeRelativeTransform(const nav_msgs::msg::Odometry::SharedPtr odom1,
                           const nav_msgs::msg::Odometry::SharedPtr odom2) {
    geometry_msgs::msg::Transform transform;

    // Compute relative translation
    transform.translation.x =
        odom2->pose.pose.position.x - odom1->pose.pose.position.x;
    transform.translation.y =
        odom2->pose.pose.position.y - odom1->pose.pose.position.y;
    transform.translation.z =
        odom2->pose.pose.position.z - odom1->pose.pose.position.z;

    // Compute relative rotation (odom2 = odom1 * relative)
    tf2::Quaternion q1, q2, q_rel;
    tf2::fromMsg(odom1->pose.pose.orientation, q1);
    tf2::fromMsg(odom2->pose.pose.orientation, q2);
    q_rel = q1.inverse() * q2;
    transform.rotation = tf2::toMsg(q_rel);

    return transform;
  }

  /**
   * @brief Apply transform to pose
   */
  void applyTransform(geometry_msgs::msg::Pose &pose,
                      const geometry_msgs::msg::Transform &transform) {
    // Apply rotation
    tf2::Quaternion q_pose, q_transform;
    tf2::fromMsg(pose.orientation, q_pose);
    tf2::fromMsg(transform.rotation, q_transform);
    tf2::Quaternion q_new = q_pose * q_transform;
    pose.orientation = tf2::toMsg(q_new);

    // Apply translation (rotated into new frame)
    tf2::Vector3 trans(transform.translation.x, transform.translation.y,
                       transform.translation.z);
    tf2::Vector3 rotated_trans = tf2::quatRotate(q_pose, trans);
    pose.position.x += rotated_trans.x();
    pose.position.y += rotated_trans.y();
    pose.position.z += rotated_trans.z();
  }
};

/**
 * @brief Point cloud distortion correction using odometry
 *  * 尝试使用laser时间片内的里程数据进行样条曲线拟合
 * 1： xyz
 * 使用CSplines，但这并不是最好，因为它不能处理旋转；对于差速轮模型，由于其是非完全模型；其平面速度方向，应该与朝向一致；
 * 2： 对于旋转使用Squad进行角速度不变平滑；
 * 3： 只针对laser帧前后的数据进行拟合；必须过数据点
 * 4： 时间片内拟合； 不关心整体连续性；
 */
class DistortionCorrector {
public:
  /**
   * @brief Correct point cloud distortion using odometry between frames
   */
  static std::vector<Point> correctDistortion(
      std::shared_ptr<FrameData> &laser_frame,
      const std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr>
          &odom_queue,
      const transforms::Rigid3d &laser_to_base) {
    if (laser_frame->between_next_odoms.empty()) {
      std::cerr << "当前laser的消息传递的消息为空" << std::endl;
      return convertScanToPoints(laser_frame->scan);
    }

    std::vector<Point> corrected_points;
    corrected_points.reserve(laser_frame->scan->ranges.size());

    // Convert scan to points first
    auto original_points = convertScanToPoints(laser_frame->scan);
    // 1. 获取laser帧前后两个里程数据
    std::vector<PosePoint> odom_poses(laser_frame->between_next_odoms.size());
    for (const auto odom_stamp : laser_frame->between_next_odoms) {
      PosePoint tmp_pose;
      tmp_pose.pose =
          transforms::ToRigid3d(odom_queue.at(odom_stamp)->pose.pose);
      tmp_pose.timestamp = odom_stamp * 1e-9;
      odom_poses.emplace_back(tmp_pose);
    }
    // 2. 统计各点的里程计时间戳
    // TODO: 当前激光雷达的原始数据并不准确；
    // 扫描事件为时间片为0；只能以time_increment进行计算
    std::vector<double> point_timestamps(original_points.size());
    int less_cnt = 0;
    int gt_cnt = 0;
    for (const auto &odom_timestamp : laser_frame->between_next_odoms) {
      if (laser_frame->timestamp < odom_timestamp)
        less_cnt++;
      else
        gt_cnt++;
    }
    std::cerr << "当前laser的消息传递前后里程计消息数量: "
              << laser_frame->between_next_odoms.size() << "负轴：" << less_cnt
              << " 个；正轴: " << gt_cnt << "个" << std::endl;
    // 3. 对于laser帧前后两个里程数据进行插值
    PoseCubicSpline odom_spline(odom_poses);
    // 4. 计算当前扫描点的里程计位姿
    auto global_base_pose =
        odom_spline.interpolate(laser_frame->timestamp * 1e-9);
    laser_frame->global_pose = global_base_pose.pose * laser_to_base;
    // 5. 计算每个点在短时里程计下的全局坐标
    int index = 0;
    for (size_t i = 0; i < laser_frame->scan->ranges.size(); ++i) {
      // 跳过无效点
      if (laser_frame->scan->ranges[i] < laser_frame->scan->range_min ||
          laser_frame->scan->ranges[i] > laser_frame->scan->range_max ||
          !std::isfinite(laser_frame->scan->ranges[i])) {
        continue;
      }
      // 计算相对于laser的点云
      Point point;
      double angle =
          laser_frame->scan->angle_min + i * laser_frame->scan->angle_increment;
      double range = laser_frame->scan->ranges[i];
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      if (i < laser_frame->scan->intensities.size()) {
        point.intensity = laser_frame->scan->intensities[i];
      } else {
        point.intensity = 0.0;
      }
      double point_stamp =
          laser_frame->timestamp * 1e-9 + i * laser_frame->scan->time_increment;
      PosePoint stamp_odom = odom_spline.interpolate(point_stamp);
      auto global_point = laser_frame->global_pose.inverse() * stamp_odom.pose *
                          laser_to_base *
                          Eigen::Vector3d(point.x, point.y, 0.0);
      // Interpolate odometry
      // auto interpolated_odom = interpolateOdometry(odom_start, odom_end,
      // alpha);

      // Transform point to world frame using interpolated odometry
      // Point corrected =
      //     transformPointToWorld(original_points[i], interpolated_odom);
      Point corrected_point;
      corrected_point.x = global_point.x();
      corrected_point.y = global_point.y();
      if (i < laser_frame->scan->intensities.size()) {
        corrected_point.intensity = laser_frame->scan->intensities[i];
      } else {
        corrected_point.intensity = 0.0;
      }
      corrected_point.origin_index = index++;
      corrected_points.push_back(corrected_point);
    }
    return corrected_points;
  }

private:
  /**
   * @brief Convert laser scan to points
   */
  static std::vector<Point>
  convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<Point> points;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      if (scan_msg->ranges[i] < scan_msg->range_min ||
          scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i])) {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      Point point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);

      if (i < scan_msg->intensities.size()) {
        point.intensity = scan_msg->intensities[i];
      } else {
        point.intensity = 0.0;
      }

      points.push_back(point);
    }

    return points;
  }

  /**
   * @brief Transform point from laser frame to world frame
   */
  static Point
  transformPointToWorld(const Point &point,
                        const geometry_msgs::msg::Pose &odom_pose) {
    Point world_point;

    // Rotate point by odometry orientation
    tf2::Quaternion q;
    tf2::fromMsg(odom_pose.orientation, q);
    tf2::Vector3 v(point.x, point.y, 0);
    tf2::Vector3 rotated = tf2::quatRotate(q, v);

    // Translate by odometry position
    world_point.x = rotated.x() + odom_pose.position.x;
    world_point.y = rotated.y() + odom_pose.position.y;
    world_point.intensity = point.intensity;

    return world_point;
  }
};

/**
 * @brief Interactive bag processing node
 */
class ReflectorNoiseBagNode : public rclcpp::Node {
public:
  ReflectorNoiseBagNode()
      : Node("reflector_noise_bag_node"), current_frame_index_(0),
        auto_mode_(false), should_exit_(false) {
    // Declare parameters
    this->declare_parameter("bag_path", "");
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("odom_topic", "/odom_combined");
    this->declare_parameter("classification_method", "pca");
    this->declare_parameter("raw_intensity_threshold", 1000.0);
    this->declare_parameter("expected_diameter", 0.07);
    this->declare_parameter("diameter_tolerance", 0.03);
    this->declare_parameter("enable_interpolation", true);
    this->declare_parameter("min_confidence", 0.5);

    // Get parameters
    bag_path_ = this->get_parameter("bag_path").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    classification_method_ =
        this->get_parameter("classification_method").as_string();
    raw_intensity_threshold_ =
        this->get_parameter("raw_intensity_threshold").as_double();
    expected_diameter_ = this->get_parameter("expected_diameter").as_double();
    diameter_tolerance_ = this->get_parameter("diameter_tolerance").as_double();
    enable_interpolation_ =
        this->get_parameter("enable_interpolation").as_bool();
    min_confidence_ = this->get_parameter("min_confidence").as_double();

    // Configure detection modules
    configureDetectionModules();

    // Create publisher for visualization
    laser_pub_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_cloud", 10);
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/reflector_cluster_cloud", 10);
    // 近似反光柱形状插值补偿
    compensated_cluster_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/reflector_compensated_cluster_cloud", 10);
    cluster_circle_points_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/reflector_cluster_circlefit_cloud", 10);
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/reflector_bag_trajectory", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/reflector_detected_markers", 10);
    tracked_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reflector_tracked_markers", 10);

    // Create timer for continuous publishing (10 Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ReflectorNoiseBagNode::publishTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "反光柱逐帧检测节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "Bag路径: %s", bag_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "扫描话题: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "里程计话题: %s", odom_topic_.c_str());
  }

  /**
   * @brief Run the bag processing
   */
  void run() {
    if (bag_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Bag路径未设置，请使用--ros-args -p bag_path:=<path>");
      return;
    }

    // Open bag file
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter_options;
    reader.open(storage_options, converter_options);
    RCLCPP_INFO(this->get_logger(), "打开Bag文件: %s", bag_path_.c_str());
    for (auto &topic : reader.get_all_topics_and_types()) {
      RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s", topic.name.c_str(),
                  topic.type.c_str());
    }

    // Pre-load all frames
    if (!loadAllFrames(reader)) {
      RCLCPP_ERROR(this->get_logger(), "加载帧失败");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "加载完成，共 %zu 帧", frames_.size());

    // Start keyboard input thread
    std::thread input_thread(&ReflectorNoiseBagNode::keyboardInputThread, this);

    // Process first frame
    processFrame(0);

    // Spin the node (this will run the timer)
    rclcpp::spin(shared_from_this());

    // Wait for input thread to finish
    should_exit_ = true;
    // input_thread.join();

    RCLCPP_INFO(this->get_logger(), "处理完成");
  }

private:
  /**
   * @brief Configure detection modules
   */
  void configureDetectionModules() {
    // Configure circle fittercircle_fitter_
    CircleFitParams circle_params;
    circle_params.max_fit_error = 0.03;
    circle_params.min_inlier_ratio = 0.5;
    circle_params.max_fit_error_near = 0.04;
    circle_params.max_fit_error_far = 0.02;
    circle_params.far_distance_threshold = 3.0;
    circle_fitter_.setParams(circle_params);

    // Configure improved interpolator
    ImprovedInterpolationCompensator::InterpolationParams interp_params;
    interp_params.min_points = 5;
    interp_params.max_points = 20;
    interp_params.min_distance = 2.0;
    interp_params.max_distance = 4.0;
    interp_params.gap_multiplier = 0.8;
    improved_interpolator_.setParams(interp_params);

    // Configure PCA classifier
    ShapeClassificationParams pca_params;
    pca_params.max_elongation_for_post = 9.50;
    pca_params.min_elongation_for_board = 12.0;
    pca_params.min_linearity_for_board = 0.93;
    pca_params.max_linearity_for_post = 0.97;
    pca_params.min_circularity_for_post = 0.64;
    pca_params.min_circularity_for_board = 0.46;
    pca_classifier_.setParams(pca_params);

    // Configure geometric validator
    geometric_validator_.setExpectedDiameter(expected_diameter_);
    geometric_validator_.setDiameterTolerance(diameter_tolerance_);
  }

  /**
   * @brief Load all frames from bag
   * WARN: 由于里程计前后时间跳变, 这里选用录包时刻的系统时间戳
   */
  bool loadAllFrames(rosbag2_cpp::Reader &reader) {
    int64_t peek_time = 0; // Peek time for next message

    auto laser_scan_serializer =
        rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
    // auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    auto tf_serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
    int64_t last_odom_stamp = 0;
    int64_t last_scan_stamp = 0;

    // Read all messages
    RCLCPP_INFO(this->get_logger(), "开始读取rosbag包: %s ..... ",
                bag_path_.c_str());
    while (reader.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (msg->time_stamp > peek_time) {
        peek_time = msg->time_stamp;
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "录制的rosbag包出现前后事件跳变: 消息 %s, %ld",
                     msg->topic_name.c_str(), msg->time_stamp);
      }
      // Process based on topic
      if (msg->topic_name == scan_topic_) {
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
        laser_scan_serializer.deserialize_message(&serialized_msg, scan.get());
        if (scan_map_.find(rclcpp::Time(msg->time_stamp).nanoseconds()) ==
            scan_map_.end()) {
          scan_map_[rclcpp::Time(msg->time_stamp).nanoseconds()] = scan;
          scan_timestamps_.push_back(
              rclcpp::Time(msg->time_stamp).nanoseconds());
          assert(last_scan_stamp < rclcpp::Time(msg->time_stamp).nanoseconds());
          last_scan_stamp = rclcpp::Time(msg->time_stamp).nanoseconds();
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "读取scan消息: "
                                  << " bag timestamp: " << msg->time_stamp
                                  << "存在重复时间戳");
        }
      } else if (msg->topic_name == odom_topic_) {
        auto odom = std::make_shared<nav_msgs::msg::Odometry>();
        odom_serializer.deserialize_message(&serialized_msg, odom.get());
        if (odom_map_.find(rclcpp::Time(odom->header.stamp).nanoseconds()) ==
            odom_map_.end()) {
          odom_map_[rclcpp::Time(odom->header.stamp).nanoseconds()] = odom;
          odom_timestamps_.push_back(
              rclcpp::Time(odom->header.stamp).nanoseconds());
          assert(last_odom_stamp <
                 rclcpp::Time(odom->header.stamp).nanoseconds());
          last_odom_stamp = rclcpp::Time(odom->header.stamp).nanoseconds();
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "读取odom消息: "
                                  << " bag timestamp: " << msg->time_stamp
                                  << "存在重复时间戳");
        }
      } else if (msg->topic_name == "/tf_static") {
        // Read tf_static for laser_scan to base_link transform
        auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        try {
          tf_serializer.deserialize_message(&serialized_msg, tf_msg.get());
          for (auto &transform : tf_msg->transforms) {
            if (transform.header.frame_id == "base_link" &&
                transform.child_frame_id == "laser") {
              laser_to_base_ = transform;
              RCLCPP_INFO(this->get_logger(),
                          "找到laser到base_link的变换: (%.3f, %.3f)",
                          transform.transform.translation.x,
                          transform.transform.translation.y);
            }
          }
        } catch (const rclcpp::exceptions::RCLError &rcl_error) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "解析TF_STATIC发生错误" << rcl_error.what());
        }
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "完成读取rosbag包:  ..... " << bag_path_);
    // 展示整体队列和信息内容:
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "激光雷达队列信息: " << scan_map_.size()
                                            << "帧, 起始范围: ["
                                            << scan_timestamps_.front() << " , "
                                            << scan_timestamps_.back() << "]");
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "里程计队列信息: " << odom_map_.size()
                                          << "帧, 起始范围: ["
                                          << scan_timestamps_.front() << " , "
                                          << scan_timestamps_.back() << "]");
    // 为激光数据进行里程计计算
    // 1. C-Splines拟合算法
    // TODO: 此处应该直接使用
    // scan的时间戳来进行{duration}时间范围内查找，可用里程计；
    // 这里使用3次样条插值进行拟合
    // 估计出laserscan当前时刻下以odom为关联轴的位姿拟合结果，并附带关联odom的起始数据，以及C-BSpline的拟合函数；
    // ---odom3--odom4--odom5--|--laser0-- | --odom6--odom7--odom9-- |
    // --laser1-- | --odom10--odom11--odom12-- |
    // --laser2 -- ... laser0: [odom3, odom9]
    // 进行数据关联，并利用此范围内数据进行拟合;
    // 当接收到laser1时，所以laser0为数据处理起始位置 laser1： [odom4, odom12]
    // 进行数据关联，并利用此范围内数据进行拟合;
    // 当接收到laser2时，永远以2帧为1拍进行数据关联
    size_t frame_idx = 0;
    std::vector<int64_t>::const_iterator odom_peek = odom_timestamps_.cbegin();
    std::sort(scan_timestamps_.begin(), scan_timestamps_.end());
    std::sort(odom_timestamps_.begin(), odom_timestamps_.end());
    for (std::vector<int64_t>::const_iterator scan_iterator =
             scan_timestamps_.cbegin();
         scan_iterator < scan_timestamps_.cend() - 1; ++scan_iterator) {
      const auto &next_scan_iterator = scan_iterator + 1;
      const auto &scan_time = *scan_iterator;
      const auto &next_scan_time = *next_scan_iterator;
      auto frame = std::make_shared<FrameData>();
      frame->scan = scan_map_[scan_time];
      frame->timestamp = scan_time;
      frame->frame_index = frame_idx++;
      // Find closest odometry
      std::vector<int64_t>::const_iterator keep_odom_peek = odom_peek;
      for (; odom_peek < odom_timestamps_.cend(); ++odom_peek) {
        if (*odom_peek <= scan_time) {
          frame->between_next_odoms.push_back(*odom_peek);
          keep_odom_peek++; // 保留当前的odom_peek，用于下一次迭代，查找相邻帧的数据
        }
        if (*odom_peek > scan_time && *odom_peek < next_scan_time) {
          frame->between_next_odoms.push_back(*odom_peek);
        }
      }
      odom_peek = keep_odom_peek;
      // 记录数据
      frames_.push_back(frame);
    }
    RCLCPP_INFO(this->get_logger(), "激光雷达数据为: %d帧", frames_.size());
    // 补偿最后一帧激光雷达的odom数据虽然它可能只有一半
    if (odom_peek < odom_timestamps_.cend()) {
      for (std::vector<int64_t>::const_iterator scan_iterator =
               scan_timestamps_.cend() - 1;
           scan_iterator < scan_timestamps_.cend(); ++scan_iterator) {
        const auto &scan_time = *scan_iterator;
        auto frame = std::make_shared<FrameData>();
        frame->scan = scan_map_[scan_time];
        frame->timestamp = scan_time;
        frame->frame_index = frame_idx++;
        for (; odom_peek < odom_timestamps_.cend(); ++odom_peek) {
          if (*odom_peek <= scan_time) {
            frame->between_next_odoms.push_back(*odom_peek);
          }
          if (*odom_peek > scan_time) {
            frame->between_next_odoms.push_back(*odom_peek);
          }
        }
        frames_.push_back(frame);
        RCLCPP_INFO(this->get_logger(),
                    "补偿激光雷达数据为: %d帧, 使用的里程计数据: %ld帧",
                    frames_.size(), frame->between_next_odoms.size());
      }
    }
    return !frames_.empty();
  }

  /**
   * @brief Process a single frame
   */
  void processFrame(size_t frame_index) {
    if (frame_index >= frames_.size()) {
      RCLCPP_WARN(this->get_logger(), "帧索引超出范围: %zu/%zu", frame_index,
                  frames_.size());
      return;
    }

    current_frame_index_ = frame_index;
    auto &frame = frames_[frame_index];

    RCLCPP_INFO(this->get_logger(), "处理帧 %zu/%zu, 时间: %.3f", frame_index,
                frames_.size() - 1, rclcpp::Time(frame->timestamp).seconds());
    // 使用扭曲补偿
    frame->compensated_points = DistortionCorrector::correctDistortion(
        frame, odom_map_, transforms::ToRigid3d(laser_to_base_));

    // Update global pose tracker
    // if (frame->odom) {
    //   pose_tracker_.update(frame->odom);
    //   frame->global_pose = pose_tracker_.getGlobalPose();
    //   pose_tracker_.addTrajectoryPoint(frame->global_pose,
    //                                    rclcpp::Time(frame->timestamp));
    // }

    // // Apply distortion correction
    // if (frame_index > 0 && frame->odom && frames_[frame_index - 1]->odom) {
    //   frame->compensated_points = DistortionCorrector::correctDistortion(
    //       frame->scan, frames_[frame_index - 1]->odom, frame->odom);
    //   RCLCPP_INFO(this->get_logger(), "应用畸变校正");
    // } else {
    //   // No previous odometry, convert directly
    //   frame->compensated_points = convertScanToPoints(frame->scan);
    // }

    // Apply intensity filtering
    auto filtered_points =
        filterByIntensity(frame->compensated_points, raw_intensity_threshold_);

    RCLCPP_INFO(this->get_logger(), "原始点数: %zu, 强度过滤后: %zu",
                frame->compensated_points.size(), filtered_points.size());

    frame->filtered_points = filtered_points;

    // DBSCAN clustering
    // TODO: 在Point中加入OrignIndex然后利用这个索引进行回溯,
    // cluster中点云以originIndex进行排序;
    auto cluster_indices = fixed_dbscan_.cluster(filtered_points);

    if (cluster_indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "DBSCAN聚类后无簇");
      frame->reflectors.clear();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "DBSCAN聚类得到 %zu 个簇",
                cluster_indices.size());
    // 连续性过滤,同样以 Point点云以 OriginIndex进行索引排序
    auto splited_clusters =
        continueClusterDetector(filtered_points, cluster_indices, 3);
    RCLCPP_INFO(this->get_logger(), "DBSCAN连续性分割后,得到 %zu 个簇",
                splited_clusters.size());
    // Convert indices to clusters
    std::vector<std::vector<Point>> clusters;
    int cluster_idx = 0;
    for (const auto &indices : splited_clusters) {
      std::vector<Point> cluster;
      cluster.reserve(indices.size());
      for (int idx : indices) {
        cluster.push_back(filtered_points[idx]);
        frame->filtered_points[idx].intensity = 2000 + cluster_idx * 200;
      }
      clusters.push_back(cluster);
      cluster_idx++;
    }

    // Detect reflectors
    // TODO: 修复圆拟合检测性问题，连续性插值检测;
    // 局部非凹性检测; 1.2m内有大噪声; 保存： 当前帧pcd点云;
    detectReflectors(clusters, frame->reflectors);

    RCLCPP_INFO(this->get_logger(), "检测到 %zu 个反光柱",
                frame->reflectors.size());

    // Visualization data is ready, will be published by timer
  }
  /**
   * @brief
   * 利用聚类的索引序列，判断聚类的连续性，如果中间有断开，则认为是两个聚类
   */
  std::vector<std::vector<int>>
  continueClusterDetector(const std::vector<Point> &filtered_points,
                          const std::vector<std::vector<int>> &cluster_indices,
                          int gap_threshold) {
    std::vector<std::vector<int>> new_cluster_indices;
    int cluster_idx = 0;
    for (const auto &indices : cluster_indices) {
      std::vector<int> new_cluster;
      for (std::vector<int>::const_iterator iter = indices.begin();
           iter < indices.end() - 1; ++iter) {
        int index_gap = filtered_points[*(iter + 1)].origin_index -
                        filtered_points[*iter].origin_index;
        // gap设置为3 超过3个重新打断分类
        if (index_gap > gap_threshold) {
          std::vector<int> new_split_cluster(new_cluster);
          if (new_split_cluster.size() > 8) {
            new_cluster_indices.push_back(new_split_cluster);
          }
          new_cluster.clear();
        }
        new_cluster.push_back(*iter);
      }
      if (new_cluster.size() > 8) {
        new_cluster_indices.push_back(new_cluster);
      }
      cluster_idx++;
    }
    return new_cluster_indices;
  }

  /**
   * @brief Detect reflectors from clusters
   */
  void detectReflectors(const std::vector<std::vector<Point>> &clusters,
                        std::vector<DetectedReflector> &reflectors) {
    reflectors.clear();
    cluster_circle_points_.clear();

    for (size_t idx = 0; idx < clusters.size(); ++idx) {
      const auto &cluster = clusters[idx];
      Point center = computeCentroid(cluster);
      double distance = center.distanceFromOrigin();

      // Classification
      bool is_reflector_candidate = false;

      if (classification_method_ == "pca") {
        auto pca_features =
            pca_classifier_.computeShapeFeatures(cluster, circle_fitter_);
        auto circle_fit_temp = circle_fitter_.fitCircle(cluster);
        // 分别根据pca信息和拟合圆信息判别是直线，还是圆弧，以及噪声
        // 噪声检测基本失败
        auto object_type =
            pca_classifier_.classifyObject(pca_features, circle_fit_temp);

        is_reflector_candidate = (object_type == REFLECTOR_POST);

        RCLCPP_INFO(this->get_logger(),
                    "簇 %zu: 延伸度=%.3f, 线性度=%.3f, 圆形度=%.3f, "
                    "聚类点数=%zu, 中心距离=%.3f, 类型=%s",
                    idx, pca_features.elongation, pca_features.linearity,
                    pca_features.circularity, cluster.size(), distance,
                    is_reflector_candidate ? "反光柱" : "其他");
      }

      if (!is_reflector_candidate) {
        continue;
      }

      // Interpolation
      // 插值补偿距离较远的稀疏点云，进行弧长插补(Depatched)
      auto processed_cluster = cluster;
      if (enable_interpolation_) {
        processed_cluster = improved_interpolator_.interpolateCluster(
            cluster, center, distance);
        for (const auto &point : processed_cluster) {
          cluster_circle_points_.push_back(point);
        }
        RCLCPP_INFO(this->get_logger(),
                    "进行插值补偿， 补偿前总点数: %zu, 补偿后总点数: %zu",
                    cluster.size(), processed_cluster.size());
      }

      // 圆拟合基本失败,修正圆拟合方法
      // auto circle_fit = circle_fitter_.fitCircle(processed_cluster);
      auto circle_fit = circle_fitter_.fitArcWithRANSAC(processed_cluster);

      // if (!circle_fitter_.validateFit(circle_fit, cluster.size(), distance))
      // {
      //   RCLCPP_INFO(this->get_logger(), "簇 %zu: 圆拟合失败", idx);
      //   continue;
      // }
      if (!circle_fit.is_valid) {
        RCLCPP_WARN(this->get_logger(), "簇 %zu: RANSAC圆拟合失败", idx);
        continue;
      }
      RCLCPP_INFO(
          this->get_logger(),
          "簇 %zu : 中心(%.3f,%.3f), 直径=%.3fm, 总误差=%.6f, "
          "内点数=%d, 内点比例=%.3f, 总点数=%d, 内点误差=%6f, 外点误差=%6f",
          idx, circle_fit.center.x, circle_fit.center.y,
          circle_fit.radius * 2.0, circle_fit.fit_error,
          circle_fit.inlier_count, circle_fit.inlier_ratio,
          circle_fit.total_points, circle_fit.inner_error, circle_fit.outline_error);
      if (circle_fitter_.validateFit(circle_fit, cluster.size(), distance)) {
        DetectedReflector reflector;
        reflector.center = circle_fit.center;
        reflector.diameter = 2 * circle_fit.radius;
        reflector.confidence = 0.8;
        reflector.point_count = cluster.size();
        reflector.idx = idx;
        reflectors.push_back(reflector);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "簇 %zu: RANSAC圆拟合圆拟合 内点误差验证失败", idx);
      }

      // Compute confidence
      // double confidence = computeConfidence(cluster, circle_fit);

      // if (confidence >= min_confidence_) {
      //   DetectedReflector reflector;
      //   reflector.center = circle_fit.center;
      //   reflector.diameter = 2 * circle_fit.radius;
      //   reflector.confidence = confidence;
      //   reflector.point_count = cluster.size();
      //   reflector.idx = idx;
      //   reflectors.push_back(reflector);

      //   RCLCPP_INFO(this->get_logger(),
      //               "反光柱 %zu: 中心(%.3f,%.3f), 直径=%.3fm, 置信度=%.3f",
      //               reflectors.size(), reflector.center.x,
      //               reflector.center.y, reflector.diameter, confidence);
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "簇 %zu: 拟合置信度太低，判定失败",
      //               idx);
      // }
    }
  }

  /**
   * @brief Compute confidence for a detected reflector
   */
  double computeConfidence([[maybe_unused]] const std::vector<Point> &cluster,
                           const CircleFitResult &circle_fit) {
    // Simple confidence based on fit quality
    double fit_quality = 1.0 - std::min(1.0, circle_fit.fit_error / 0.03);
    double inlier_quality = circle_fit.inlier_ratio;

    return 0.6 * fit_quality + 0.4 * inlier_quality;
  }

  /**
   * @brief Timer callback for continuous publishing
   */
  void publishTimerCallback() {
    if (current_frame_index_ >= frames_.size()) {
      return;
    }

    const auto &frame = frames_[current_frame_index_];

    pulishOriginLaserScan(frame->scan);

    // Publish point cloud with current timestamp
    publishPointCloud(frame->compensated_points);

    // 发布阈值滤波后的点云
    publishFilteredPointCloud(frame->filtered_points);

    // 发布进行圆形拟合后的点云
    publishClusterCircleFitPointCloud(this->cluster_circle_points_);

    // Publish reflector markers with current timestamp
    // 新增可视化拟合圆
    publishReflectorMarkers(frame->reflectors);

    // Publish trajectory with current timestamp
    publishTrajectory();
  }

  /**
   * @brief Keyboard input thread
   */
  void keyboardInputThread() {
    RCLCPP_INFO(this->get_logger(), "键盘控制:");
    RCLCPP_INFO(this->get_logger(), "  'n' - 下一帧");
    RCLCPP_INFO(this->get_logger(), "  'p' - 上一帧");
    RCLCPP_INFO(this->get_logger(), "  ' ' (空格) - 切换自动模式");
    RCLCPP_INFO(this->get_logger(), "  'q' - 退出");

    while (!should_exit_) {
      char key = get_char_without_enter();
      RCLCPP_INFO(this->get_logger(), "Key: %c", key);
      switch (key) {
      case 'n':
      case 'N':
        if (current_frame_index_ < frames_.size() - 1) {
          processFrame(current_frame_index_ + 1);
        } else {
          RCLCPP_WARN(this->get_logger(), "已是最后一帧");
        }
        break;

      case 'p':
      case 'P':
        if (current_frame_index_ > 0) {
          processFrame(current_frame_index_ - 1);
        } else {
          RCLCPP_WARN(this->get_logger(), "已是第一帧");
        }
        break;

      case ' ':
        auto_mode_ = !auto_mode_;
        RCLCPP_INFO(this->get_logger(), "自动模式: %s",
                    auto_mode_ ? "开启" : "关闭");
        if (auto_mode_) {
          // startAutoMode();
        }
        break;

      case 'q':
      case 'Q':
        RCLCPP_INFO(this->get_logger(), "安全退出程序");
        should_exit_ = true;
        rclcpp::shutdown();
        break;

      default:
        break;
      }
    }
  }

  /**
   * @brief Start automatic processing mode
   */
  void startAutoMode() {
    std::thread([this]() {
      while (auto_mode_ && !should_exit_) {
        if (current_frame_index_ < frames_.size() - 1) {
          processFrame(current_frame_index_ + 1);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
          RCLCPP_INFO(this->get_logger(), "自动处理完成");
          auto_mode_ = false;
          break;
        }
      }
    }).detach();
  }

  void
  pulishOriginLaserScan(const sensor_msgs::msg::LaserScan::Ptr &laser_msg) {
    laser_msg->header.stamp = this->now();
    laser_pub_->publish(*laser_msg);
  }
  /**
   * @brief Publish point cloud with current timestamp
   */
  void publishPointCloud(const std::vector<Point> &points) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "laser";
    cloud_msg.height = 1;
    cloud_msg.width = points.size();

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);

    for (size_t i = 0; i < points.size(); ++i) {
      uint8_t *ptr = &cloud_msg.data[i * cloud_msg.point_step];

      float x = static_cast<float>(points[i].x);
      std::memcpy(ptr + 0, &x, sizeof(float));

      float y = static_cast<float>(points[i].y);
      std::memcpy(ptr + 4, &y, sizeof(float));

      float z = 0.0f;
      std::memcpy(ptr + 8, &z, sizeof(float));

      float intensity = static_cast<float>(points[i].intensity);
      std::memcpy(ptr + 12, &intensity, sizeof(float));
    }

    cluster_pub_->publish(cloud_msg);
  }

  void publishFilteredPointCloud(const std::vector<Point> &points) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "laser";
    cloud_msg.height = 1;
    cloud_msg.width = points.size();

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);

    for (size_t i = 0; i < points.size(); ++i) {
      uint8_t *ptr = &cloud_msg.data[i * cloud_msg.point_step];

      float x = static_cast<float>(points[i].x);
      std::memcpy(ptr + 0, &x, sizeof(float));

      float y = static_cast<float>(points[i].y);
      std::memcpy(ptr + 4, &y, sizeof(float));

      float z = 0.0f;
      std::memcpy(ptr + 8, &z, sizeof(float));

      float intensity = static_cast<float>(points[i].intensity);
      std::memcpy(ptr + 12, &intensity, sizeof(float));
    }

    filtered_pub_->publish(cloud_msg);
  }

  void publishClusterCircleFitPointCloud(const std::vector<Point> &points) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "laser";
    cloud_msg.height = 1;
    cloud_msg.width = points.size();

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);

    for (size_t i = 0; i < points.size(); ++i) {
      uint8_t *ptr = &cloud_msg.data[i * cloud_msg.point_step];

      float x = static_cast<float>(points[i].x);
      std::memcpy(ptr + 0, &x, sizeof(float));

      float y = static_cast<float>(points[i].y);
      std::memcpy(ptr + 4, &y, sizeof(float));

      float z = 0.0f;
      std::memcpy(ptr + 8, &z, sizeof(float));

      float intensity = static_cast<float>(points[i].intensity);
      std::memcpy(ptr + 12, &intensity, sizeof(float));
    }

    cluster_circle_points_pub_->publish(cloud_msg);
  }

  /**
   * @brief Publish reflector markers with current timestamp
   */
  void
  publishReflectorMarkers(const std::vector<DetectedReflector> &reflectors) {
    visualization_msgs::msg::MarkerArray marker_array;
    // Resize to accommodate both cylinder markers and text labels
    marker_array.markers.resize(reflectors.size() * 3);

    for (size_t i = 0; i < reflectors.size(); ++i) {
      // Create cylinder marker for the reflector
      visualization_msgs::msg::Marker cylinder_marker;
      cylinder_marker.header.stamp = this->now();
      cylinder_marker.header.frame_id = "laser";
      cylinder_marker.ns = "reflective_posts";
      cylinder_marker.id = i * 3; // Even IDs for cylinders
      cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      cylinder_marker.action = visualization_msgs::msg::Marker::ADD;

      cylinder_marker.pose.position.x = reflectors[i].center.x;
      cylinder_marker.pose.position.y = reflectors[i].center.y;
      cylinder_marker.pose.position.z = 0.0;
      cylinder_marker.pose.orientation.w = 1.0;

      // marker.scale.x = reflectors[i].diameter;
      // marker.scale.y = reflectors[i].diameter;
      cylinder_marker.scale.x = 0.001;
      cylinder_marker.scale.y = 0.001;
      cylinder_marker.scale.z = 0.5;

      double confidence = reflectors[i].confidence;
      cylinder_marker.color.r = 0.0;
      cylinder_marker.color.g = confidence;
      cylinder_marker.color.b = 1.0 - confidence;
      cylinder_marker.color.a = 0.8;

      cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

      marker_array.markers[i * 3] = cylinder_marker;

      // 创建检测出来的拟合圆
      visualization_msgs::msg::Marker cylinder_marker2;
      cylinder_marker2.header.stamp = this->now();
      cylinder_marker2.header.frame_id = "laser";
      cylinder_marker2.ns = "reflective_posts";
      cylinder_marker2.id = i * 3 + 1; // Even IDs for cylinders
      cylinder_marker2.type = visualization_msgs::msg::Marker::CYLINDER;
      cylinder_marker2.action = visualization_msgs::msg::Marker::ADD;

      cylinder_marker2.pose.position.x = reflectors[i].center.x;
      cylinder_marker2.pose.position.y = reflectors[i].center.y;
      cylinder_marker2.pose.position.z = 0.0;
      cylinder_marker2.pose.orientation.w = 1.0;

      // marker.scale.x = reflectors[i].diameter;
      // marker.scale.y = reflectors[i].diameter;
      cylinder_marker2.scale.x = reflectors[i].diameter;
      cylinder_marker2.scale.y = reflectors[i].diameter;
      cylinder_marker2.scale.z = 0.5;

      cylinder_marker2.color.r = 1.0;
      cylinder_marker2.color.g = 0.0;
      cylinder_marker2.color.b = 1.0;
      cylinder_marker2.color.a = 0.8;

      cylinder_marker2.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers[i * 3 + 1] = cylinder_marker2;

      // Create text label marker for the reflector idx
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.stamp = this->now();
      text_marker.header.frame_id = "laser";
      text_marker.ns = "reflective_posts";
      text_marker.id = i * 3 + 2; // Odd IDs for text labels
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;

      text_marker.pose.position.x = reflectors[i].center.x;
      text_marker.pose.position.y = reflectors[i].center.y;
      text_marker.pose.position.z = 0.6; // Position text above the cylinder
      text_marker.pose.orientation.w = 1.0;

      text_marker.scale.z = 0.3; // Text height

      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0;
      text_marker.color.a = 1.0;

      // Set text to display the reflector idx
      text_marker.text = "R" + std::to_string(reflectors[i].idx);

      text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

      marker_array.markers[i * 3 + 2] = text_marker;
    }

    marker_pub_->publish(marker_array);
  }

  /**
   * @brief Publish trajectory
   */
  void publishTrajectory() {
    const auto &trajectory = pose_tracker_.getTrajectory();
    if (trajectory.empty()) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.resize(trajectory.size());
    for (size_t i = 0; i < trajectory.size(); ++i) {
      marker.points[i].x = trajectory[i].pose.position.x;
      marker.points[i].y = trajectory[i].pose.position.y;
      marker.points[i].z = 0.0;
    }

    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    trajectory_pub_->publish(marker);
  }

  /**
   * @brief Convert laser scan to points
   */
  std::vector<Point>
  convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<Point> points;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      if (scan_msg->ranges[i] < scan_msg->range_min ||
          scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i])) {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      Point point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);

      if (i < scan_msg->intensities.size()) {
        point.intensity = scan_msg->intensities[i];
      } else {
        point.intensity = 0.0;
      }

      points.push_back(point);
    }

    return points;
  }

  /**
   * @brief Filter points by intensity
   */
  std::vector<Point> filterByIntensity(const std::vector<Point> &points,
                                       double threshold) {
    std::vector<Point> filtered;
    filtered.reserve(points.size());

    for (const auto &point : points) {
      if (point.intensity >= threshold) {
        filtered.push_back(point);
      }
    }

    return filtered;
  }

  /**
   * @brief Compute centroid of cluster
   */
  Point computeCentroid(const std::vector<Point> &cluster) {
    Point centroid;
    if (cluster.empty()) {
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &point : cluster) {
      sum_x += point.x;
      sum_y += point.y;
    }

    centroid.x = sum_x / cluster.size();
    centroid.y = sum_y / cluster.size();
    return centroid;
  }

  // Parameters
  std::string bag_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string classification_method_;
  double raw_intensity_threshold_;
  double expected_diameter_;
  double diameter_tolerance_;
  bool enable_interpolation_;
  double min_confidence_;
  // 激光与里程计相关数据
  std::unordered_map<int64_t, sensor_msgs::msg::LaserScan::SharedPtr>
      scan_map_; // Map of scan messages by timestamp
  std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr> odom_map_;
  std::vector<int64_t> odom_timestamps_; // Vector of frames
  std::vector<int64_t> scan_timestamps_;

  // Frame data
  std::vector<std::shared_ptr<FrameData>> frames_;
  size_t current_frame_index_;

  // Transform
  geometry_msgs::msg::TransformStamped laser_to_base_;

  // Pose tracking
  PoseTracker pose_tracker_;

  // Detection modules
  FixedDBSCAN fixed_dbscan_;
  FractalDimensionCalculator fd_calculator_;
  PCAShapeClassifier pca_classifier_;
  ImprovedInterpolationCompensator improved_interpolator_;
  std::vector<Point> cluster_circle_points_;
  CircleFitter circle_fitter_;
  PracticalDescriptorExtractor descriptor_extractor_;
  GeometricValidator geometric_validator_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      tracked_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      compensated_cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cluster_circle_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;

  // Timer for continuous publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Threading
  std::atomic<bool> auto_mode_;
  std::atomic<bool> should_exit_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReflectorNoiseBagNode>();

  RCLCPP_INFO(node->get_logger(), "启动反光柱逐帧检测 (Bag处理版本)");

  node->run();

  rclcpp::shutdown();

  return 0;
}