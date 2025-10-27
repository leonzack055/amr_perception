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
    double squaredNorm() const {
        return x*x + y*y;
    }
};
// 增强的车辆运动状态
struct VehicleState {
    Vector2d position;           // 车辆位置
    Vector2d velocity;           // 车辆速度
    float orientation;          // 朝向角度
    float angular_velocity;     // 角速度
    Vector2d center_point;       // 反光条中心点
    Vector2d center_velocity;    // 中心点速度
    
    VehicleState() : orientation(0), angular_velocity(0) {}
};

// 扩展卡尔曼滤波器 - 处理大角度运动
class ExtendedKalmanFilter {
private:
    VehicleState state_;
    
    // 协方差矩阵 (简化版)
    float pos_variance_[2][2];    // 位置协方差
    float vel_variance_[2][2];    // 速度协方差  
    float angle_variance_;
    float angular_vel_variance_;
    
    const float process_noise_ = 1e-4f;
    const float measurement_noise_ = 1e-3f;
    bool initialized_;
    
public:
    ExtendedKalmanFilter() : angle_variance_(1.0f), angular_vel_variance_(1.0f), initialized_(false) {
        // 初始化协方差矩阵
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                pos_variance_[i][j] = (i == j) ? 1.0f : 0.0f;
                vel_variance_[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }
    
    void initialize(const Vector2d& position, float orientation) {
        state_.position = position;
        state_.orientation = orientation;
        state_.velocity = Vector2d(0, 0);
        state_.angular_velocity = 0;
        state_.center_point = position;
        state_.center_velocity = Vector2d(0, 0);
        
        initialized_ = true;
    }
    
    // 预测步骤 - 使用自行车模型
    void predict(float dt) {
        if (!initialized_) return;
        
        // 自行车运动模型预测
        predictBicycleModel(dt);
        
        // 增加过程噪声
        for (int i = 0; i < 2; ++i) {
            pos_variance_[i][i] += process_noise_;
            vel_variance_[i][i] += process_noise_;
        }
        angle_variance_ += process_noise_;
        angular_vel_variance_ += process_noise_;
    }
    
    // 更新步骤
    void update(const Vector2d& measured_center, float measured_orientation, float dt) {
        if (!initialized_) {
            initialize(measured_center, measured_orientation);
            return;
        }
        
        // 中心点更新
        updateCenter(measured_center, dt);
        
        // 角度更新  
        updateOrientation(measured_orientation, dt);
    }
    
    const VehicleState& getState() const { return state_; }
    
private:
    void predictBicycleModel(float dt) {
        // 简化的自行车模型
        // 假设车辆绕后轴旋转
        
        // 位置预测
        state_.position = state_.position + state_.velocity * dt;
        
        // 角度预测
        state_.orientation += state_.angular_velocity * dt;
        state_.orientation = normalizeAngle(state_.orientation);
        
        // 中心点预测（相对于车辆位置）
        float wheelbase = 2.5f; // 假设轴距
        Vector2d local_center(0, wheelbase * 0.5f); // 假设中心在前轴和后轴之间
        
        // 将局部坐标转换到世界坐标系
        float cos_theta = std::cos(state_.orientation);
        float sin_theta = std::sin(state_.orientation);
        
        state_.center_point = Vector2d(
            state_.position.x + local_center.x * cos_theta - local_center.y * sin_theta,
            state_.position.y + local_center.x * sin_theta + local_center.y * cos_theta
        );
    }
    
    void updateCenter(const Vector2d& measured_center, float dt) {
        // 计算创新向量
        Vector2d innovation = measured_center - state_.center_point;
        
        // 简化的卡尔曼增益
        float gain = 0.3f; // 固定增益，实际应该基于协方差
        
        // 状态更新
        state_.center_point = state_.center_point + innovation * gain;
        state_.center_velocity = innovation * (gain / dt);
        
        // 更新车辆位置（基于中心点和朝向）
        updateVehiclePositionFromCenter();
    }
    
    void updateOrientation(float measured_orientation, float dt) {
        float innovation = normalizeAngle(measured_orientation - state_.orientation);
        
        // 动态增益：大角度变化时使用较小增益
        float gain = (std::abs(innovation) > 0.5f) ? 0.1f : 0.3f;
        
        state_.orientation = normalizeAngle(state_.orientation + innovation * gain);
        state_.angular_velocity = innovation * (gain / dt);
    }
    
    void updateVehiclePositionFromCenter() {
        // 根据中心点和朝向反推车辆位置
        float wheelbase = 2.5f;
        Vector2d local_center(0, wheelbase * 0.5f);
        
        float cos_theta = std::cos(state_.orientation);
        float sin_theta = std::sin(state_.orientation);
        
        state_.position = Vector2d(
            state_.center_point.x - (local_center.x * cos_theta - local_center.y * sin_theta),
            state_.center_point.y - (local_center.x * sin_theta + local_center.y * cos_theta)
        );
    }
    
    float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
};

// 大角度运动处理器
class LargeAngleMotionProcessor {
private:
    ExtendedKalmanFilter ekf_;
    Vector2d radar_position_;
    
    // 运动状态检测
    bool is_turning_;
    float turning_threshold_;
    std::deque<float> recent_angular_velocities_;
    
    // 历史状态用于插值
    std::deque<VehicleState> state_history_;
    const int history_size_ = 10;
    
public:
    LargeAngleMotionProcessor(const Vector2d& radar_pos = Vector2d(0, 0)) 
        : radar_position_(radar_pos), is_turning_(false), turning_threshold_(0.3f) {}
    
    struct StableResult {
        Vector2d stable_center;
        float stable_orientation;
        Vector2d stable_direction;
        bool is_turning;
        float turning_intensity;
        float confidence;
    };
    
    StableResult processWithLargeAngleSupport(const std::vector<Vector2d>& radar_points, 
                                            float dt) {
        StableResult result;
        result.confidence = 0;
        
        if (radar_points.size() < 3) {
            return result;
        }
        
        // 1. 提取当前帧的原始信息
        Vector2d raw_center = computeCenter(radar_points);
        //float raw_orientation = computeOrientation(radar_points, radar_position_);
        float raw_orientation = computePerpendicularOrientation(radar_points, radar_position_);
        // 2. 检测大角度运动
        detectTurningMotion(raw_orientation, dt);
        
        // 3. EKF预测
        ekf_.predict(dt);
        
        // 4. EKF更新（使用自适应参数）
        ekf_.update(raw_center, raw_orientation, dt);
        
        // 5. 获取稳定状态
        const VehicleState& state = ekf_.getState();
        
        // 6. 根据运动状态调整输出
        if (is_turning_) {
            result = processTurningState(state, dt);
        } else {
            result = processStraightState(state);
        }
        
        // 7. 保存历史状态
        saveStateHistory(state);
        
        return result;
    }
    
private:
    Vector2d computeCenter(const std::vector<Vector2d>& points) {
        Vector2d center(0, 0);
        for (const auto& p : points) center = center + p;
        return center * (1.0f / points.size());
    }
    float computePerpendicularOrientation(const std::vector<Vector2d>& points, const Vector2d& radar_pos) {
        Vector2d center = computeCenter(points);
        
        // 使用PCA计算反光条的主方向
        float sum_xx = 0, sum_xy = 0, sum_yy = 0;
        for (const auto& p : points) {
            Vector2d vec = p - center;
            sum_xx += vec.x * vec.x;
            sum_xy += vec.x * vec.y;
            sum_yy += vec.y * vec.y;
        }
        
        // 计算反光条的主方向（直线方向）
        Vector2d line_direction(sum_xy, sum_yy - sum_xx);
        line_direction = line_direction.normalized();
        
        // 计算垂直于反光条的两个可能方向
        Vector2d normal1(-line_direction.y, line_direction.x);  // 逆时针旋转90度
        Vector2d normal2(line_direction.y, -line_direction.x);  // 顺时针旋转90度
        
        // 选择指向雷达的方向
        Vector2d to_radar = radar_pos - center;
        Vector2d final_normal = (normal1.dot(to_radar) > normal2.dot(to_radar)) ? normal1 : normal2;
        
        return std::atan2(final_normal.y, final_normal.x);
    }
    float computeOrientation(const std::vector<Vector2d>& points, const Vector2d& radar_pos) {
        // 简化的方向计算
        Vector2d center = computeCenter(points);
        
        // 使用PCA计算主方向
        float sum_xx = 0, sum_xy = 0, sum_yy = 0;
        for (const auto& p : points) {
            Vector2d vec = p - center;
            sum_xx += vec.x * vec.x;
            sum_xy += vec.x * vec.y;
            sum_yy += vec.y * vec.y;
        }
        
        // 计算法线方向（指向雷达）
        Vector2d normal(sum_xy, sum_yy - sum_xx);
        normal = normal.normalized();
        
        // 确保指向雷达
        Vector2d to_radar = radar_pos - center;
        if (normal.dot(to_radar) < 0) {
            normal = normal * (-1.0f);
        }
        
        return std::atan2(normal.y, normal.x);
    }
    
    void detectTurningMotion(float current_orientation, float dt) {
        // 保存最近的角速度
        static float last_orientation = current_orientation;
        float angular_velocity = normalizeAngle(current_orientation - last_orientation) / dt;
        last_orientation = current_orientation;
        
        recent_angular_velocities_.push_back(angular_velocity);
        if (recent_angular_velocities_.size() > 5) {
            recent_angular_velocities_.pop_front();
        }
        
        // 计算平均角速度
        float avg_angular_velocity = 0;
        for (float w : recent_angular_velocities_) {
            avg_angular_velocity += w;
        }
        avg_angular_velocity /= recent_angular_velocities_.size();
        
        // 检测转向状态
        is_turning_ = std::abs(avg_angular_velocity) > turning_threshold_;
    }
    
    StableResult processTurningState(const VehicleState& state, float dt) {
        StableResult result;
        
        // 转向状态下使用更强的平滑和预测
        result.stable_center = state.center_point;
        result.stable_orientation = state.orientation;
        result.stable_direction = Vector2d(std::cos(state.orientation), std::sin(state.orientation));
        result.is_turning = true;
        result.turning_intensity = std::abs(state.angular_velocity);
        
        // 转向时置信度降低
        result.confidence = std::max(0.3f, 1.0f - result.turning_intensity * 0.5f);
        
        return result;
    }
    
    StableResult processStraightState(const VehicleState& state) {
        StableResult result;
        
        // 直线状态下使用标准处理
        result.stable_center = state.center_point;
        result.stable_orientation = state.orientation;
        result.stable_direction = Vector2d(std::cos(state.orientation), std::sin(state.orientation));
        result.is_turning = false;
        result.turning_intensity = 0;
        result.confidence = 0.9f;
        
        return result;
    }
    
    void saveStateHistory(const VehicleState& state) {
        state_history_.push_back(state);
        if (state_history_.size() > history_size_) {
            state_history_.pop_front();
        }
    }
    
    float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
};
// 并查集用于优化DBSCAN
class UnionFind {
private:
    std::vector<int> parent;
    std::vector<int> rank;
public:
    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }
    
    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }
    
    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
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

        /*publisher_debug = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/charger_relative_pose_debug1", 1);*/
        
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/charger_relative_pose", 1);

        RCLCPP_INFO(this->get_logger(), "反光条检测节点初始化完成");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_debug;
    LargeAngleMotionProcessor estimator;
    
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
    // 计算两点之间的欧氏距离
    double distanceTo(const Vector2d& current_points,const Vector2d& other){
        double dx = current_points.x - other.x;
        double dy = current_points.y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 轻量级EPS自适应
    double compute_fast_adaptive_eps(const std::vector<Vector2d>& points, double base_eps) {
        if (points.size() < 10) return base_eps;
        
        // 快速计算点云密度
        Vector2d min_pt = points[0], max_pt = points[0];
        for (const auto& p : points) {
            min_pt.x = std::min(min_pt.x, p.x);
            min_pt.y = std::min(min_pt.y, p.y);
            max_pt.x = std::max(max_pt.x, p.x);
            max_pt.y = std::max(max_pt.y, p.y);
        }
        
        double area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);
        double density = points.size() / (area + 1e-6);
        
        // 简单密度自适应
        double adaptive_factor = 1.0;
        if (density < 0.1) adaptive_factor = 1.3;  // 稀疏点云，增大EPS
        else if (density > 10.0) adaptive_factor = 0.7;  // 密集点云，减小EPS
        
        return base_eps * adaptive_factor;
    }

    // 预计算网格邻域关系
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> 
    precompute_grid_neighbors(const std::map<std::pair<int, int>, std::vector<int>>& grid) {
        std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> neighbors;
        
        for (const auto& [grid_coord, _] : grid) {
            auto [x, y] = grid_coord;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    neighbors[{x, y}].push_back({x + dx, y + dy});
                }
            }
        }
        return neighbors;
    }
  
	// 自适应 DBSCAN 聚类
	std::vector<int> adaptive_dbscan(const std::vector<Vector2d>& points) {
		int min_cluster_points = this->get_parameter("min_cluster_points").as_int();
		double cluster_eps = this->get_parameter("cluster_eps").as_double();
        if (int(points.size()) < min_cluster_points) {
		    return std::vector<int>(points.size(), -1);
		}
		
		double base_eps = cluster_eps;
        
        // 1. 快速EPS自适应（避免复杂计算）
        double eps = compute_fast_adaptive_eps(points, base_eps);
        
        // 2. 保持网格划分，但优化搜索策略
        double grid_size = eps;
        std::map<std::pair<int, int>, std::vector<int>> grid;
        
        for (size_t i = 0; i < points.size(); ++i) {
            int grid_x = static_cast<int>(points[i].x / grid_size);
            int grid_y = static_cast<int>(points[i].y / grid_size);
            grid[{grid_x, grid_y}].push_back(i);
        }
        
        // 3. 优化：预计算网格邻域关系
        auto grid_neighbors = precompute_grid_neighbors(grid);
        
        UnionFind uf(points.size());
        std::vector<bool> is_core(points.size(), false);
        std::vector<int> neighbor_counts(points.size(), 0);
        
        // 4. 第一阶段：快速计数
        #pragma omp parallel for if(points.size() > 100)
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& current_point = points[i];
            int grid_x = static_cast<int>(current_point.x / grid_size);
            int grid_y = static_cast<int>(current_point.y / grid_size);
            
            int count = 0;
            // 只搜索相邻的9个网格
            for (const auto& neighbor_grid : grid_neighbors.at({grid_x, grid_y})) {
                auto it = grid.find(neighbor_grid);
                if (it != grid.end()) {
                    for (int idx : it->second) {
                        if ((current_point - points[idx]).squaredNorm() < eps * eps) {
                            count++;
                            if (count >= min_cluster_points) break;
                        }
                    }
                }
                if (count >= min_cluster_points) break;
            }
            neighbor_counts[i] = count;
        }
        
        // 5. 第二阶段：核心点判定和合并（优化合并策略）
        for (size_t i = 0; i < points.size(); ++i) {
            if (neighbor_counts[i] >= min_cluster_points) {
                is_core[i] = true;
                
                const auto& current_point = points[i];
                int grid_x = static_cast<int>(current_point.x / grid_size);
                int grid_y = static_cast<int>(current_point.y / grid_size);
                
                // 只与邻近的核心点合并，减少合并操作
                for (const auto& neighbor_grid : grid_neighbors.at({grid_x, grid_y})) {
                    auto it = grid.find(neighbor_grid);
                    if (it != grid.end()) {
                        for (int idx : it->second) {
                            if (is_core[idx] && 
                                (current_point - points[idx]).squaredNorm() < eps * eps) {
                                uf.unite(i, idx);
                            }
                        }
                    }
                }
            }
        }
        
        // 6. 快速标签分配（保持原有逻辑）
        std::vector<int> labels(points.size(), -1);
        std::map<int, int> cluster_map;
        int next_cluster_id = 0;
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (is_core[i]) {
                int root = uf.find(i);
                if (cluster_map.find(root) == cluster_map.end()) {
                    cluster_map[root] = next_cluster_id++;
                }
                labels[i] = cluster_map[root];
            }
        }
        
        // 7. 简化边界点分配：单次遍历
        for (size_t i = 0; i < points.size(); ++i) {
            if (!is_core[i] && labels[i] == -1) {
                int grid_x = static_cast<int>(points[i].x / grid_size);
                int grid_y = static_cast<int>(points[i].y / grid_size);
                
                for (const auto& neighbor_grid : grid_neighbors.at({grid_x, grid_y})) {
                    auto it = grid.find(neighbor_grid);
                    if (it != grid.end()) {
                        for (int idx : it->second) {
                            if (labels[idx] != -1 && 
                                (points[i] - points[idx]).squaredNorm() < eps * eps) {
                                labels[i] = labels[idx];
                                break;
                            }
                        }
                    }
                    if (labels[i] != -1) break;
                }
            }
        }
        
        return labels;
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

        /*geometry_msgs::msg::PoseStamped best_pose_debug;
        best_pose_debug.header = msg->header;  // 更新时间戳
        best_pose_debug.pose.position.x = 0.0;
        best_pose_debug.pose.position.y = 0.0;*/
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
            RCLCPP_DEBUG(this->get_logger(), "未检测到高强度点，不发布结果");
            return;
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
            RCLCPP_DEBUG(this->get_logger(), "未检测到有效聚类，不发布结果");
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
                auto result = estimator.processWithLargeAngleSupport(cluster, time_stamp_);
                
                double theta = result.stable_orientation;
                Vector2d ori_ref(result.stable_center.x,result.stable_center.y);
                std::vector<double> ori_coor;
                ori_coor.push_back(ori_ref.x);
                ori_coor.push_back(ori_ref.y);
                ori_coor.push_back(theta);
                /*best_pose_debug.pose.position.x = ori_coor[0];
                best_pose_debug.pose.position.y = ori_coor[1];
                best_pose_debug.pose.orientation.z = std::sin(ori_coor[2] / 2);
                best_pose_debug.pose.orientation.w = std::cos(ori_coor[2] / 2);
                //outFile1<< best_pose_debug.pose.position.x <<","<<best_pose_debug.pose.position.y <<","<<theta<<std::endl;
                RCLCPP_INFO(this->get_logger(),
                    "发布radar_to_reflector结果debug：中心(%.3fm, %.3fm), 方向=%.3frad",
                    ori_coor[0], ori_coor[1], ori_coor[2]);*/
                std::vector<double> car_r = transformCarToReflectorFrame(ori_coor);

                best_pose.pose.position.x = car_r[0];
                best_pose.pose.position.y = car_r[1];
                best_pose.pose.orientation.z = std::sin(car_r[2] / 2);
                best_pose.pose.orientation.w = std::cos(car_r[2] / 2);
                /*double theta_result = 2 * std::atan2(best_pose.pose.orientation.z, best_pose.pose.orientation.w);
                RCLCPP_INFO(this->get_logger(),
                    "发布base_link_to_reflector结果：中心(%.3fm, %.3fm), 方向=%.3frad",
                    best_pose.pose.position.x, best_pose.pose.position.y, theta_result);*/
            }else{
                RCLCPP_DEBUG(this->get_logger(), "The reflective tape fails to meet the length requirement.");
            }
                
        }
        publisher_->publish(best_pose);
        //publisher_debug->publish(best_pose_debug);
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
