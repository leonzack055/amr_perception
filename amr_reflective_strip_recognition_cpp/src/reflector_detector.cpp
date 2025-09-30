#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <array>

using namespace std::chrono_literals;

//std::ofstream outFile1("./test_cpp_reflector_2.txt");

struct Vector2d {
    double x, y;
    Vector2d() : x(0), y(0) {}
    Vector2d(double x, double y) : x(x), y(y) {}
    
    Vector2d operator+(const Vector2d& other) const {
        return Vector2d(x + other.x, y + other.y);
    }
    
    Vector2d operator-(const Vector2d& other) const {
        return Vector2d(x - other.x, y - other.y);
    }
    
    Vector2d operator/(double scalar) const {
        return Vector2d(x / scalar, y / scalar);
    }

    Vector2d operator*(double scalar) const {
        return Vector2d(x * scalar, y * scalar);
    }
    
     // 叉积
    double cross(const Vector2d& other) const {
        return x * other.y - y * other.x;
    }

    Vector2d& operator+=(const Vector2d& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    double norm() const{
        return std::sqrt(x*x + y*y);
    }
    
    // 向量长度
    double length() const {
        return std::sqrt(x * x + y * y);
    }
    
    // 单位向量
    Vector2d normalized() const {
        double len = length();
        if (len < 1e-6) return Vector2d(0, 0);
        return Vector2d(x / len, y / len);
    }
    // 点积
    double dot(const Vector2d& other) const {
        return x * other.x + y * other.y;
    }
};
// 直线结构体
struct Line {
    double a, b, c; // 直线方程: ax + by + c = 0
    Vector2d center; // 中心点
    double length;  // 直线长度估计
    std::vector<Vector2d> points; // 原始点
    
    Line(double a = 0, double b = 0, double c = 0, Vector2d center = Vector2d(), double len = 0) 
        : a(a), b(b), c(c), center(center), length(len) {}
};

// 雷达数据处理器
class RadarDataProcessor {
public:
    // 从雷达原始数据提取直线信息（改进版本）
    static Line extractLineFromRadarData(const std::vector<Vector2d>& radar_points) {
        if (radar_points.size() < 2) {
            return Line();
        }
        
        // 计算中心点
        Vector2d center(0, 0);
        for (const auto& point : radar_points) {
            center.x += point.x;
            center.y += point.y;
        }
        center.x /= radar_points.size();
        center.y /= radar_points.size();
        
        // 计算边界点和长度估计
        double max_dist = 0;
        Vector2d farthest1, farthest2;
        for (const auto& p1 : radar_points) {
            for (const auto& p2 : radar_points) {
                double dist = (p1 - p2).length();
                if (dist > max_dist) {
                    max_dist = dist;
                    farthest1 = p1;
                    farthest2 = p2;
                }
            }
        }
        
        // 使用RANSAC-like方法拟合直线，提高鲁棒性
        Line best_line = fitLineRobust(radar_points, center);
        best_line.points = radar_points;
        best_line.length = max_dist;
        
        return best_line;
    }
    
    // 鲁棒直线拟合
    static Line fitLineRobust(const std::vector<Vector2d>& points, const Vector2d& center) {
        if (points.size() < 2) return Line();
        
        // 方法1: 使用PCA主成分分析
        double sum_xx = 0, sum_xy = 0, sum_yy = 0;
        for (const auto& p : points) {
            double dx = p.x - center.x;
            double dy = p.y - center.y;
            sum_xx += dx * dx;
            sum_xy += dx * dy;
            sum_yy += dy * dy;
        }
        
        // 计算特征向量（主方向）
        double theta = 0.5 * std::atan2(2 * sum_xy, sum_xx - sum_yy);
        
        // 直线方向向量
        Vector2d dir(std::cos(theta), std::sin(theta));
        
        // 转换为一般式: -sin(theta)*x + cos(theta)*y + (sin(theta)*cx - cos(theta)*cy) = 0
        double a = -dir.y;
        double b = dir.x;
        double c = dir.y * center.x - dir.x * center.y;
        
        return Line(a, b, c, center);
    }
    
    // 计算垂直于直线的方向，并确保朝向雷达（改进版本）
    static double calculatePerpendicularDirection(const Line& line, const Vector2d& radar_position = Vector2d(0, 0)) {
        // 直线的法向量有两个可能方向: (a, b) 和 (-a, -b)
        Vector2d normal_vector(line.a, line.b);
        normal_vector = normal_vector.normalized();
        
        // 方法1: 使用点云密度判断（更稳定）
        double density_score1 = calculateDirectionScore(line.points, normal_vector, radar_position);
        double density_score2 = calculateDirectionScore(line.points, normal_vector * (-1.0), radar_position);
        
        // 选择得分更高的方向（更可能朝向雷达的方向）
        Vector2d final_direction = (density_score1 > density_score2) ? normal_vector : normal_vector * (-1.0);
        
        // 计算方向角度
        double angle = std::atan2(final_direction.y, final_direction.x);
        
        return angle;
    }
    
private:
    // 计算方向得分（基于点云密度和雷达位置）
    static double calculateDirectionScore(const std::vector<Vector2d>& points, 
                                         const Vector2d& direction, 
                                         const Vector2d& radar_position) {
        if (points.empty()) return 0;
        
        double score = 0;
        int count = 0;
        
        // 计算中心点
        Vector2d center(0, 0);
        for (const auto& p : points) {
            center = center + p;
        }
        center = center / points.size();
        
        // 在给定方向上采样点，计算与雷达的距离
        for (const auto& point : points) {
            // 测试点沿着方向移动一小段距离
            Vector2d test_point = point + direction * 0.1;
            double dist_to_radar = (test_point - radar_position).length();
            
            // 距离越小，得分越高
            score += 1.0 / (1.0 + dist_to_radar);
            count++;
        }
        
        return (count > 0) ? score / count : 0;
    }
};

// 增强的角度卡尔曼滤波器
class EnhancedAngleKalmanFilter {
private:
    struct State {
        double angle;          // 朝向角
        double angular_velocity; // 角速度
        double angular_acceleration; // 角加速度
    };
    
    State state;
    double angle_variance;
    double angular_velocity_variance;
    double angular_acceleration_variance;
    double process_noise;
    double measurement_noise;
    bool is_initialized;
    double max_angle_change;
    
    // 历史数据用于平滑
    std::deque<double> angle_history;
    const size_t history_size = 5;
    
public:
    EnhancedAngleKalmanFilter(double max_change_deg = 15.0,  // 更严格的限制
                            double proc_noise = 0.0001,     // 更小的过程噪声
                            double meas_noise = 0.005)      // 更小的测量噪声
        : process_noise(proc_noise), measurement_noise(meas_noise), 
          is_initialized(false), max_angle_change(max_change_deg * M_PI / 180.0) {
        state = {0, 0, 0};
        angle_variance = 1.0;
        angular_velocity_variance = 0.1;
        angular_acceleration_variance = 0.01;
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    void initialize(double initial_angle) {
        state.angle = normalizeAngle(initial_angle);
        state.angular_velocity = 0;
        state.angular_acceleration = 0;
        angle_variance = 0.01;  // 更小的初始方差
        angular_velocity_variance = 0.001;
        angular_acceleration_variance = 0.0001;
        
        angle_history.clear();
        angle_history.push_back(state.angle);
        
        is_initialized = true;
    }
    
    void predict(double dt) {
        if (!is_initialized) return;
        
        // 更精确的状态预测（恒定角加速度模型）
        state.angle += state.angular_velocity * dt + 0.5 * state.angular_acceleration * dt * dt;
        state.angle = normalizeAngle(state.angle);
        state.angular_velocity += state.angular_acceleration * dt;
        
        // 协方差预测
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        
        angle_variance += dt2 * angular_velocity_variance + 0.25 * dt3 * angular_acceleration_variance + process_noise;
        angular_velocity_variance += dt * angular_acceleration_variance + process_noise;
    }
    
    bool update(double measurement, double dt) {
        if (!is_initialized) {
            initialize(measurement);
            return true;
        }
        
        double normalized_measurement = normalizeAngle(measurement);
        
        // 改进的野值检测：使用历史趋势预测
        double predicted_angle = state.angle + state.angular_velocity * dt;
        double angle_diff = normalizeAngle(normalized_measurement - predicted_angle);
        
        // 动态阈值：高速时允许更大变化
        double dynamic_threshold = max_angle_change * (1.0 + std::abs(state.angular_velocity) * 0.1);
        
        if (std::abs(angle_diff) > dynamic_threshold) {
            return false; // 野值，不更新
        }
        
        // 卡尔曼增益
        double kalman_gain = angle_variance / (angle_variance + measurement_noise);
        
        // 状态更新
        state.angle = normalizeAngle(state.angle + kalman_gain * angle_diff);
        
        // 协方差更新
        angle_variance = (1.0 - kalman_gain) * angle_variance;
        
        // 更新角速度和角加速度估计
        if (dt > 1e-6) {
            // 使用历史数据平滑角速度估计
            angle_history.push_back(state.angle);
            if (angle_history.size() > history_size) {
                angle_history.pop_front();
            }
            
            // 基于多个历史点计算角速度
            if (angle_history.size() >= 3) {
                double weighted_velocity = 0;
                double total_weight = 0;
                
                for (size_t i = 1; i < angle_history.size(); ++i) {
                    double delta_angle = normalizeAngle(angle_history[i] - angle_history[i-1]);
                    double weight = i; // 越近的点权重越大
                    weighted_velocity += (delta_angle / dt) * weight;
                    total_weight += weight;
                }
                
                if (total_weight > 0) {
                    double new_angular_velocity = weighted_velocity / total_weight;
                    // 低通滤波
                    double alpha = 0.3;
                    state.angular_velocity = (1 - alpha) * state.angular_velocity + alpha * new_angular_velocity;
                }
            }
        }
        
        return true;
    }
    
    // 获取平滑后的角度（使用移动平均）
    double getSmoothedAngle() const {
        if (angle_history.empty()) return state.angle;
        
        // 使用向量平均方法计算角度平均值
        double sum_sin = 0, sum_cos = 0;
        for (double angle : angle_history) {
            sum_sin += std::sin(angle);
            sum_cos += std::cos(angle);
        }
        
        return std::atan2(sum_sin, sum_cos);
    }
    
    double getAngle() const { 
        return getSmoothedAngle(); // 返回平滑后的角度
    }
    
    double getAngularVelocity() const { return state.angular_velocity; }
    bool initialized() const { return is_initialized; }
};

// 多帧一致性检查器
class ConsistencyChecker {
private:
    std::deque<double> recent_angles;
    const size_t check_window = 3;
    double max_variance;
    
public:
    ConsistencyChecker(double max_var_deg = 5.0) 
        : max_variance(max_var_deg * M_PI / 180.0) {}
    
    bool checkConsistency(double new_angle) {
        recent_angles.push_back(new_angle);
        if (recent_angles.size() > check_window) {
            recent_angles.pop_front();
        }
        
        if (recent_angles.size() < check_window) return true;
        
        // 计算角度方差
        double mean_sin = 0, mean_cos = 0;
        for (double angle : recent_angles) {
            mean_sin += std::sin(angle);
            mean_cos += std::cos(angle);
        }
        mean_sin /= recent_angles.size();
        mean_cos /= recent_angles.size();
        
        double variance = 1.0 - std::sqrt(mean_sin * mean_sin + mean_cos * mean_cos);
        
        return variance < max_variance;
    }
    
    void reset() {
        recent_angles.clear();
    }
};

// 主车辆朝向估计器（改进版本）
class StableVehicleOrientationEstimator {
private:
    std::unique_ptr<EnhancedAngleKalmanFilter> angle_filter;
    ConsistencyChecker consistency_checker;
    double last_timestamp;
    bool first_frame;
    int consecutive_outliers;
    const int max_consecutive_outliers = 3; // 更严格
    Vector2d radar_position;
    
    // 历史结果用于平滑
    std::deque<double> smoothed_angles;
    const size_t smooth_window = 3;
    
public:
    StableVehicleOrientationEstimator(const Vector2d& radar_pos = Vector2d(0, 0)) 
        : last_timestamp(0.0), first_frame(true), consecutive_outliers(0), 
          radar_position(radar_pos) {
        angle_filter = std::make_unique<EnhancedAngleKalmanFilter>();
    }
    
    struct OrientationResult {
        Vector2d center_point;
        double orientation_angle; // 弧度
        double orientation_angle_deg; // 度
        bool is_stable;
        std::string status;
        Vector2d direction_vector;
        double confidence; // 置信度 [0,1]
    };
    
    OrientationResult processRadarData(const std::vector<Vector2d>& radar_points, double timestamp) {
        OrientationResult result;
        result.confidence = 1.0;
        
        // 从雷达数据提取直线信息
        Line detected_line = RadarDataProcessor::extractLineFromRadarData(radar_points);
        
        // 计算垂直于直线且朝向雷达的方向
        double raw_angle = RadarDataProcessor::calculatePerpendicularDirection(detected_line, radar_position);
        
        if (first_frame) {
            angle_filter->initialize(raw_angle);
            last_timestamp = timestamp;
            first_frame = false;
            
            result.center_point = detected_line.center;
            result.orientation_angle = raw_angle;
            result.orientation_angle_deg = raw_angle * 180.0 / M_PI;
            result.direction_vector = Vector2d(std::cos(raw_angle), std::sin(raw_angle));
            result.is_stable = true;
            result.status = "Initialized";
            
            smoothed_angles.push_back(raw_angle);
            return result;
        }
        
        double dt = timestamp - last_timestamp;
        last_timestamp = timestamp;
        
        if (dt <= 0 || dt > 1.0) {
            dt = 0.033;
        }
        
        // 预测步骤
        angle_filter->predict(dt);
        
        // 一致性检查
        bool is_consistent = consistency_checker.checkConsistency(raw_angle);
        
        // 更新步骤（只有通过一致性检查才更新）
        bool update_success = false;
        if (is_consistent) {
            update_success = angle_filter->update(raw_angle, dt);
        }
        
        // 处理异常情况
        if (!is_consistent || !update_success) {
            consecutive_outliers++;
            result.status = "Unstable: " + std::string(!is_consistent ? "Inconsistent" : "Outlier") + 
                          " (" + std::to_string(consecutive_outliers) + " consecutive)";
            result.is_stable = (consecutive_outliers <= max_consecutive_outliers);
            result.confidence = std::max(0.0, 1.0 - consecutive_outliers * 0.3);
        } else {
            consecutive_outliers = 0;
            result.status = "Stable";
            result.is_stable = true;
        }
        
        // 获取滤波后的结果
        double filtered_angle = angle_filter->getAngle();
        
        // 最终平滑：使用移动平均
        smoothed_angles.push_back(filtered_angle);
        if (smoothed_angles.size() > smooth_window) {
            smoothed_angles.pop_front();
        }
        
        // 计算最终平滑角度
        double final_angle = computeSmoothedAngle(smoothed_angles);
        
        result.center_point = detected_line.center;
        result.orientation_angle = final_angle;
        result.orientation_angle_deg = final_angle * 180.0 / M_PI;
        result.direction_vector = Vector2d(std::cos(final_angle), std::sin(final_angle));
        
        return result;
    }
    
private:
    double computeSmoothedAngle(const std::deque<double>& angles) {
        if (angles.empty()) return 0;
        
        double sum_sin = 0, sum_cos = 0;
        for (double angle : angles) {
            sum_sin += std::sin(angle);
            sum_cos += std::cos(angle);
        }
        
        return std::atan2(sum_sin, sum_cos);
    }
};

class ReflectorDetector : public rclcpp::Node
{
public:
    ReflectorDetector() : Node("reflector_detector")
    {
        // 参数声明
        this->declare_parameter("intensity_threshold_use", 2000);
        this->declare_parameter("cluster_eps", 0.1);
        this->declare_parameter("min_cluster_points", 20);
        this->declare_parameter("diameter_min", 0.09);
        this->declare_parameter("diameter_max", 0.11);
        this->declare_parameter("time_stamp", 0.033);
        this->declare_parameter("stat_mean_k", 10);
        this->declare_parameter("stat_std_threshold", 1.0);

        this->declare_parameter("lidar_to_base_tx", 0.35);
        this->declare_parameter("lidar_to_base_ty", 0.0);
        this->declare_parameter("lidar_to_base_tz", 3.1416);

        // 订阅和发布
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&ReflectorDetector::scan_callback, this, std::placeholders::_1));

        publisher_debug = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/charger_relative_pose_debug1", 1);
        
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/charger_relative_pose1", 1);

        RCLCPP_INFO(this->get_logger(), "反光条检测节点初始化完成");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_debug;
    StableVehicleOrientationEstimator estimator;
    
    // 角度归一化到 [-π, π]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // 将小车从雷达坐标系转换到反光条坐标系
    std::vector<double> transformCarToReflectorFrame(std::vector<double> reflector_radar_pose) {
        // 反光条中心点在雷达坐标系下的位置和朝向
        double X_reflect = reflector_radar_pose[0];
        double Y_reflect = reflector_radar_pose[1];
        double theta_reflect = reflector_radar_pose[2];
        
        double std_threshold_lidar_x = this->get_parameter("lidar_to_base_tx").as_double();
        double std_threshold_lidar_y = this->get_parameter("lidar_to_base_ty").as_double();
        double std_threshold_lidar_z = this->get_parameter("lidar_to_base_tz").as_double(); 
        // 小车在雷达坐标系下的位置和朝向
        double X_car = std_threshold_lidar_x;
        double Y_car = std_threshold_lidar_y;
        double theta_car = std_threshold_lidar_z;
        
        // 计算相对于反光条中心的偏移量
        double deltaX = X_car - X_reflect;
        double deltaY = Y_car - Y_reflect;
        
        // 反光条坐标系定义：
        // X_reflect轴：垂直于反光条方向（即检测到的朝向）
        // Y_reflect轴：平行于反光条方向（即朝向+90°）
        
        // 旋转变换：将偏移向量旋转到反光条坐标系
        double cos_theta = cos(theta_reflect);
        double sin_theta = sin(theta_reflect);
        
        // 将全局坐标转换到反光条坐标系
        double x_reflect = deltaX * cos_theta + deltaY * sin_theta;
        double y_reflect = -deltaX * sin_theta + deltaY * cos_theta;
        
        // 计算小车在反光条坐标系下的朝向
        double theta_reflect_frame = normalizeAngle(theta_car - theta_reflect);
        std::vector<double> result_;
        result_.push_back(x_reflect);
        result_.push_back(y_reflect);
        result_.push_back(theta_reflect_frame);
        return result_;
    }

    // 统计离群点过滤
    std::vector<Vector2d> statistical_outlier_filter(const std::vector<Vector2d>& points,int points_num)
    {
        int mean_k = this->get_parameter("stat_mean_k").as_int();
        double std_threshold = this->get_parameter("stat_std_threshold").as_double();
        if (int(points.size()) < points_num) return points;

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
            int count = std::min(mean_k, static_cast<int>(distances.size()));
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

        std::vector<Vector2d> filtered;
        for (size_t i = 0; i < points.size(); ++i) {
            if (mean_distances[i] < mean + std_threshold * std_dev) {
                filtered.push_back(points[i]);
            }
        }
        return filtered;
    }
    // 计算两点之间的欧氏距离
    double distanceTo(const Vector2d& current_points,const Vector2d& other){
        double dx = current_points.x - other.x;
        double dy = current_points.y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    // 计算点集的 k 近邻距离
	std::vector<double> compute_knn_distances(const std::vector<Vector2d>& points, int k) {
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
	std::vector<int> dbscan(const std::vector<Vector2d>& points, double eps, int min_samples) {
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
	std::vector<int> adaptive_dbscan(const std::vector<Vector2d>& points) {
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

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int intensity_thresh = this->get_parameter("intensity_threshold_use").as_int();
        int min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();

        // 提取高强度点
        std::vector<Vector2d> points;
        std::vector<float> high_intensities;
        geometry_msgs::msg::PoseStamped best_pose;
        best_pose.header = msg->header;  // 更新时间戳
        best_pose.pose.position.x = 0.0;
        best_pose.pose.position.y = 0.0;

        geometry_msgs::msg::PoseStamped best_pose_debug;
        best_pose_debug.header = msg->header;  // 更新时间戳
        best_pose_debug.pose.position.x = 0.0;
        best_pose_debug.pose.position.y = 0.0;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->intensities[i] > intensity_thresh &&
                msg->ranges[i] > msg->range_min &&
                msg->ranges[i] < msg->range_max &&
                !std::isnan(msg->ranges[i])) {
                
                double angle = msg->angle_min + i * msg->angle_increment;
                Vector2d tmp_p;
                tmp_p.x = msg->ranges[i] * std::cos(angle);
                tmp_p.y = msg->ranges[i] * std::sin(angle);
                points.push_back(tmp_p);
                high_intensities.push_back(msg->intensities[i]);
            }
        }
        if (points.empty()) {
            RCLCPP_INFO(this->get_logger(), "未检测到高强度点，不发布结果");
            return;
        }
        // 去噪
        if (int(points.size()) > min_cluster_points_) {
            points = statistical_outlier_filter(points,min_cluster_points_);
        }

        // 聚类
        std::vector<int> labels;
        if (int(points.size()) >= min_cluster_points_) {
            labels = adaptive_dbscan(points);
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
        unique_labels.erase(std::unique(unique_labels.begin(), unique_labels.end()), unique_labels.end());

        double diameter_min = this->get_parameter("diameter_min").as_double();
        double diameter_max = this->get_parameter("diameter_max").as_double();
        double time_stamp_ = this->get_parameter("time_stamp").as_double();
        for (int label : unique_labels) {
            std::vector<Vector2d> cluster;
            for (size_t i = 0; i < valid_labels.size(); ++i) {
                if (valid_labels[i] == label) {
                    cluster.push_back(valid_points[i]);
                }
            }
            double dis_clusters = distanceTo(cluster[0],cluster[int(cluster.size())-1]);
            if(dis_clusters > diameter_min && dis_clusters < diameter_max){
                auto result = estimator.processRadarData(cluster, time_stamp_);
                
                /*Vector2d radar_position(0.0,0.0);
                // 计算原始角度用于比较
                Line raw_line = RadarDataProcessor::extractLineFromRadarData(cluster);
                double raw_angle = RadarDataProcessor::calculatePerpendicularDirection(raw_line, radar_position);
                std::cout << "log info: \t"
                    << (raw_angle * 180.0 / M_PI) << "°\t"
                    << result.orientation_angle_deg << "°\t"
                    << "  Center: (" << result.center_point.x << ", " 
                    << result.center_point.y << ")\t"
                    << (result.is_stable ? "Yes" : "No") << "\t"
                    << (result.confidence * 100) << "%" << std::endl;*/
                
                double theta = result.orientation_angle_deg/180*M_PI;
                Vector2d ori_ref(result.center_point.x,result.center_point.y);
                std::vector<double> ori_coor;
                ori_coor.push_back(ori_ref.x);
                ori_coor.push_back(ori_ref.y);
                ori_coor.push_back(theta);
                best_pose_debug.pose.position.x = ori_coor[0];
                best_pose_debug.pose.position.y = ori_coor[1];
                best_pose_debug.pose.orientation.z = std::sin(ori_coor[2] / 2);
                best_pose_debug.pose.orientation.w = std::cos(ori_coor[2] / 2);
                //outFile1<< best_pose_debug.pose.position.x <<","<<best_pose_debug.pose.position.y <<","<<theta<<std::endl;
                RCLCPP_INFO(this->get_logger(),
                    "发布radar_to_reflector结果debug：中心(%.3fm, %.3fm), 方向=%.3frad",
                    ori_coor[0], ori_coor[1], ori_coor[2]);
                std::vector<double> car_r = transformCarToReflectorFrame(ori_coor);

                best_pose.pose.position.x = car_r[0];
                best_pose.pose.position.y = car_r[1];
                best_pose.pose.orientation.z = std::sin(car_r[2] / 2);
                best_pose.pose.orientation.w = std::cos(car_r[2] / 2);
                double theta_result = 2 * std::atan2(best_pose.pose.orientation.z, best_pose.pose.orientation.w);
                RCLCPP_INFO(this->get_logger(),
                    "发布base_link_to_reflector结果：中心(%.3fm, %.3fm), 方向=%.3frad",
                    best_pose.pose.position.x, best_pose.pose.position.y, theta_result);
            }else{
                RCLCPP_INFO(this->get_logger(), "changdu bumanzu");
            }
                
        }
        publisher_->publish(best_pose);
        publisher_debug->publish(best_pose_debug);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReflectorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
