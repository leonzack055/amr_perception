#ifndef REFLECTOR_DISTORT_CORRECTOR_H_
#define REFLECTOR_DISTORT_CORRECTOR_H_
#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"
#include "amr_reflector_noise_handling/common/time_order_queue.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <stack>
#include <unordered_map>
#include <vector>

namespace amr_reflector_noise_handling {

using TimeRigid3d = TimestampedData<transforms::Rigid3d>; // 带时间戳的位姿点类型

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
 * @brief Point cloud distortion correction using odometry
 *  * 尝试使用laser时间片内的里程数据进行样条曲线拟合
 *  *
 * 矫正点云过程，同时更新laser扫描时刻激光雷达点云，以及激光雷达在里程计下的全局位姿
 * 1： xyz
 * 使用CSplines，但这并不是最好，因为它不能处理旋转；对于差速轮模型，由于其是非完全模型；其平面速度方向，应该与朝向一致；
 * 2： 对于旋转使用Squad进行角速度不变平滑；
 * 3： 只针对laser帧前后的数据进行拟合；必须过数据点
 * 4： 时间片内拟合； 不关心整体连续性；
 */
class CublicDistortionCorrector {
public:
  /**
   * @brief Correct point cloud distortion using odometry between frames
   * @param scan_points: 带有时间戳信息的扫描点云
   * @param odom_queue: 里程计队列，它为扫描点云时间片起止前后的里程计位姿
   * @param laser_to_base： 激光雷达到base的位姿，往往是静态TF发布的结果
   */
  static std::vector<Point>
  correctDistortion(std::vector<TimePoint> &scan_points,
                    const std::vector<TimeRigid3d> &odom_queue,
                    const transforms::Rigid3d &laser_to_base,
                    int64_t scan_start, transforms::Rigid3d &laser_pose_in_odom) {
    if (odom_queue.empty()) {
      std::vector<Point> points;
      points.reserve(scan_points.size());
      for (const auto &timePoint : scan_points) {
        points.push_back(timePoint);
      }
      return points;
    }
    std::vector<Point> corrected_points;
    corrected_points.reserve(scan_points.size());

    // 1. 获取laser帧前后两个里程数据
    std::vector<PosePoint> odom_poses(odom_queue.size());
    for (const auto odom_stamp : odom_queue) {
      PosePoint tmp_pose;
      tmp_pose.pose = odom_stamp.data;
      tmp_pose.timestamp = odom_stamp.timestamp * 1e-9;
      odom_poses.emplace_back(tmp_pose);
    }
    // 2. 对于laser帧前后两个里程C-Spline曲线拟合
    PoseCubicSpline odom_spline(odom_poses);
    // TODO: 测试scan_points.front()与header.stamp的里程计位姿
    // 激光雷达在里程计下的全局坐标位姿
    auto global_base_pose = odom_spline.interpolate(scan_start * 1e-9);
    // 激光雷达在里程计下的全局坐标位姿
    laser_pose_in_odom = global_base_pose.pose * laser_to_base;
    // 3. 计算每个点在短时里程计下的全局坐标
    for (size_t i = 0; i < scan_points.size(); ++i) {
      // 计算相对于laser的点云
      TimePoint point = scan_points[i];
      double point_stamp = point.timestamp * 1e-9;
      PosePoint stamp_odom = odom_spline.interpolate(point_stamp);
      auto corrected_laser_point = laser_pose_in_odom.inverse() *
                                   stamp_odom.pose * laser_to_base *
                                   Eigen::Vector3d(point.x, point.y, 0.0);
      TimePoint corrected_point;
      corrected_point = point;
      corrected_point.x = corrected_laser_point.x();
      corrected_point.y = corrected_laser_point.y();
      corrected_point.origin_index = i;
      corrected_points.push_back(corrected_point);
    }
    return corrected_points;
  }
};
} // namespace amr_reflector_noise_handling

#endif // !REFLECTOR_DISTORT_CORRECTOR_H_