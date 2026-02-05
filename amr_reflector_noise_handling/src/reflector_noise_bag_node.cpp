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
#include "amr_reflector_noise_handling/common/time_order_queue.hpp"
#include "amr_reflector_noise_handling/distort_corrector.hpp"
#include "amr_reflector_noise_handling/fixed_dbscan.hpp"
#include "amr_reflector_noise_handling/fractal_dimension.hpp"
#include "amr_reflector_noise_handling/geometric_validator.hpp"
#include "amr_reflector_noise_handling/global_reflector_tracker.hpp"
#include "amr_reflector_noise_handling/improved_interpolation.hpp"
#include "amr_reflector_noise_handling/pca_shape_classifier.hpp"
#include "amr_reflector_noise_handling/practical_descriptor.hpp"
#include "amr_reflector_noise_handling/reflector_tracker.hpp"
#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/msg_conversion.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"
#include "amr_reflector_noise_handling/visualization_helper.hpp"
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

/**
 * @brief Frame data structure for storing scan and odometry information
 */
struct FrameData {
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  int64_t timestamp; // nanoseconds
  std::vector<int64_t> between_next_odoms;
  std::vector<TimeRigid3d> between_odoms;
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
             &odom_queue,
         const transforms::Rigid3d &laser_to_base) {
    // TODO: 使用OdometryQueue进行LaserScan数据的CSplines拟合
    // 1. 获取laser帧前后两个里程数据
    std::vector<PosePoint> odom_poses(laser_frame->between_next_odoms.size());
    for (const auto odom_stamp : laser_frame->between_next_odoms) {
      PosePoint tmp_pose;
      tmp_pose.pose =
          transforms::ToRigid3d(odom_queue.at(odom_stamp)->pose.pose);
      tmp_pose.timestamp = odom_stamp * 1e-9;
      odom_poses.emplace_back(tmp_pose);
    }
    // 3. 对于laser帧前后两个里程数据进行插值
    PoseCubicSpline odom_spline(odom_poses);
    // 4. 计算当前扫描点的里程计位姿
    auto global_base_pose =
        odom_spline.interpolate(laser_frame->timestamp * 1e-9);
    // 激光雷达在里程计下的全局坐标位姿
    global_pose_ = global_base_pose.pose * laser_to_base;
    trajectory_.push_back(global_pose_);
    // 2.
    // 使用CSpline进行插值求取laser帧各个扫描点的全局位姿；构建filtered点云
    if (!initialized_) {
      // Initialize with first odometry
      initialized_ = true;
      return;
    }
  }

  /**
   * @brief Get current global pose
   */
  transforms::Rigid3d getGlobalPose() const { return global_pose_; }

  const std::vector<transforms::Rigid3d> &getTrajectory() const {
    return trajectory_;
  }
  /**
   * @brief Reset tracker
   */
  void reset() { initialized_ = false; }

private:
  bool initialized_;
  transforms::Rigid3d global_pose_;
  std::vector<transforms::Rigid3d> trajectory_;
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
    this->declare_parameter("enable_interpolation", false);
    this->declare_parameter("min_confidence", 0.5);

    // 配置点云矫正器
    configureDistortionCorrector();
    // 配置聚类器
    configureFixedDBSCAN();

    // Global tracking parameters
    this->declare_parameter("global_tracking.match_distance_threshold", 0.3);
    this->declare_parameter("global_tracking.match_distance_inactive", 0.5);
    this->declare_parameter("global_tracking.confirm_time_window", 1.0);
    this->declare_parameter("global_tracking.min_detections_in_window", 6);
    this->declare_parameter("global_tracking.inactive_timeout", 5.0);
    this->declare_parameter("global_tracking.max_inactive_time", 60.0);
    this->declare_parameter("global_tracking.position_filter_alpha", 0.3);
    this->declare_parameter("global_tracking.position_filter_beta", 0.2);
    this->declare_parameter("global_tracking.min_std_dev", 0.02);
    this->declare_parameter("global_tracking.max_std_dev", 0.5);
    this->declare_parameter("global_tracking.min_confidence_to_track", 0.3);
    this->declare_parameter("global_tracking.confidence_filter_alpha", 0.2);
    this->declare_parameter("global_tracking.diameter_filter_alpha", 0.3);

    // Get parameters
    bag_path_ = this->get_parameter("bag_path").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    classification_method_ =
        this->get_parameter("classification_method").as_string();
    raw_intensity_threshold_ =
        this->get_parameter("raw_intensity_threshold").as_double();
    enable_interpolation_ =
        this->get_parameter("enable_interpolation").as_bool();
    min_confidence_ = this->get_parameter("min_confidence").as_double();

    // Configure global tracking
    GlobalReflectorTracker::Config tracking_config;
    tracking_config.match_distance_threshold =
        this->get_parameter("global_tracking.match_distance_threshold")
            .as_double();
    tracking_config.match_distance_inactive =
        this->get_parameter("global_tracking.match_distance_inactive")
            .as_double();
    tracking_config.confirm_time_window =
        this->get_parameter("global_tracking.confirm_time_window").as_double();
    tracking_config.min_detections_in_window =
        this->get_parameter("global_tracking.min_detections_in_window")
            .as_int();
    tracking_config.inactive_timeout =
        this->get_parameter("global_tracking.inactive_timeout").as_double();
    tracking_config.max_inactive_time =
        this->get_parameter("global_tracking.max_inactive_time").as_double();
    tracking_config.position_filter_alpha =
        this->get_parameter("global_tracking.position_filter_alpha")
            .as_double();
    tracking_config.position_filter_beta =
        this->get_parameter("global_tracking.position_filter_beta").as_double();
    tracking_config.min_std_dev =
        this->get_parameter("global_tracking.min_std_dev").as_double();
    tracking_config.max_std_dev =
        this->get_parameter("global_tracking.max_std_dev").as_double();
    tracking_config.min_confidence_to_track =
        this->get_parameter("global_tracking.min_confidence_to_track")
            .as_double();
    tracking_config.confidence_filter_alpha =
        this->get_parameter("global_tracking.confidence_filter_alpha")
            .as_double();
    tracking_config.diameter_filter_alpha =
        this->get_parameter("global_tracking.diameter_filter_alpha")
            .as_double();

    global_reflector_tracker_ = GlobalReflectorTracker(tracking_config);

    // Configure detection modules
    configureDetectionModules();

    RCLCPP_INFO(this->get_logger(), "反光柱逐帧检测节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "Bag路径: %s", bag_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "扫描话题: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "里程计话题: %s", odom_topic_.c_str());
  }

  /**
   * @brief Run the bag processing
   */
  void run() {
    // 创建反光柱可视化器
    visualization_helper_ =
        std::make_shared<VisualizationHelper>(shared_from_this());
    // Create timer for continuous publishing (10 Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ReflectorNoiseBagNode::publishTimerCallback, this));

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

    // Configure PCA classifier
    ShapeClassificationParams pca_params;
    pca_params.max_elongation_for_post = 9.50;
    pca_params.min_elongation_for_board = 12.0;
    pca_params.min_linearity_for_board = 0.93;
    pca_params.max_linearity_for_post = 0.97;
    pca_classifier_.setParams(pca_params);
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
        scan->header.stamp = rclcpp::Time(msg->time_stamp);
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
        odom->header.stamp = rclcpp::Time(msg->time_stamp);
        if (odom_map_.find(rclcpp::Time(odom->header.stamp).nanoseconds()) ==
            odom_map_.end()) {
          odom_map_[rclcpp::Time(odom->header.stamp).nanoseconds()] = odom;
          odom_timestamps_.push_back(
              rclcpp::Time(odom->header.stamp).nanoseconds());
          assert(last_odom_stamp <
                 rclcpp::Time(odom->header.stamp).nanoseconds());
          last_odom_stamp = rclcpp::Time(odom->header.stamp).nanoseconds();
          odom_queue_.push(TimeRigid3d(transforms::ToRigid3d(odom->pose.pose),
                                       last_odom_stamp));
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

    // 激光时间片间队列双拍提取
    frame_idx = 0;
    for (std::vector<int64_t>::const_iterator scan_iterator =
             scan_timestamps_.cbegin();
         scan_iterator < scan_timestamps_.cend() - 1; ++scan_iterator) {
      const auto &next_scan_iterator = scan_iterator + 1;
      const auto &scan_time = *scan_iterator;
      const auto &next_scan_time = *next_scan_iterator;
      auto frame = std::make_shared<FrameData>();
      // Find closest odometry
      auto before_vec = odom_queue_.popBefore(scan_time);
      auto after_vec = odom_queue_.getRange(scan_time, next_scan_time);
      for (const auto &odom : before_vec) {
        frames_[frame_idx]->between_odoms.emplace_back(odom);
      }
      for (const auto &odom : after_vec) {
        frames_[frame_idx]->between_odoms.emplace_back(odom);
      }
      // 打印双拍提取信息
      // RCLCPP_INFO(this->get_logger(),
      //             "[-]激光雷达第 %d帧, 使用的里程计数据: %ld帧, 双拍提取数据:
      //             "
      //             "%ld帧, before_vec: %ld, after_vec: %ld",
      //             frame_idx, frames_[frame_idx]->between_next_odoms.size(),
      //             frames_[frame_idx]->between_odoms.size(),
      //             before_vec.size(), after_vec.size());
      frame_idx++;
    }
    // 补偿最后一帧激光雷达的数据丢弃，队列中始终有一帧数据
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
    // 1. 使用扭曲补偿, 并获取扫描时刻插值轨迹
    auto scan_points = convertScanToTimedPoints(frame->scan);
    auto compensated_points = CublicDistortionCorrector::correctDistortion(
        scan_points, frame->between_odoms,
        transforms::ToRigid3d(laser_to_base_), frame->timestamp,
        frame->global_pose);
    if (!use_distort_corrector_) {
      frame->compensated_points = convertScanToPoints(frame->scan);
    } else {
      frame->compensated_points = compensated_points;
    }
    // 2. Apply intensity filtering
    auto filtered_points =
        filterByIntensity(frame->compensated_points, raw_intensity_threshold_);
    RCLCPP_INFO(this->get_logger(), "原始点数: %zu, 强度过滤后: %zu",
                frame->compensated_points.size(), filtered_points.size());
    // 3. DBSCAN clustering
    frame->filtered_points = filtered_points;
    std::vector<std::vector<Point>> clusters =
        fixed_dbscan_.splitCluster(frame->filtered_points);
    if (clusters.empty()) {
      RCLCPP_WARN(this->get_logger(), "DBSCAN聚类后无簇");
      frame->reflectors.clear();
      return;
    }

    // 4. Detect reflectors
    // INFO: 修复圆拟合检测性问题，连续性插值检测;
    // 局部非凹性检测; 1.2m内有大噪声; 保存： 当前帧pcd点云;
    detectReflectors(clusters, frame->reflectors);
    RCLCPP_INFO(this->get_logger(), "检测到 %zu 个反光柱",
                frame->reflectors.size());

    // 5. Update global reflector tracker
    global_reflector_tracker_.update(frame->reflectors, frame->global_pose,
                                     frame->timestamp);
    // Get confirmed reflectors for tracking
    // INFO: 获取匹配后的反光柱；
    // 这里获取的是短时内全部的已经确定为真实的激活反光柱；对于想看到
    // 经过跟踪过滤后反光柱位置和判定情况的情况下，需要再重新获取。
    auto confirmed_reflectors =
        global_reflector_tracker_.getConfirmedReflectors();
    RCLCPP_INFO(this->get_logger(), "全局跟踪: 已确认 %zu 个反光柱",
                confirmed_reflectors.size());

    // Visualization data is ready, will be published by timer
    for (auto &reflector : confirmed_reflectors) {
      auto local_postion = frame->global_pose.inverse() *
                           Eigen::Vector3d(reflector.global_position.x,
                                           reflector.global_position.y, 0.0);
      reflector.global_position.x = local_postion.x();
      reflector.global_position.y = local_postion.y();
    }
    auto confirmed_detected_reflectors =
        confirmDetetedReflectors(frame->reflectors, confirmed_reflectors);
    frame->reflectors = confirmed_detected_reflectors;
  }

  /**
   * @brief Detect reflectors from clusters
   */
  void detectReflectors(const std::vector<std::vector<Point>> &clusters,
                        std::vector<DetectedReflector> &reflectors) {
    reflectors.clear();

    for (size_t idx = 0; idx < clusters.size(); ++idx) {
      const auto &cluster = clusters[idx];
      Point center = computeCentroid(cluster);
      double distance = center.distanceFromOrigin();

      // Classification
      bool is_reflector_candidate = false;

      if (classification_method_ == "pca") {
        auto pca_features = pca_classifier_.computeShapeFeatures(cluster);
        auto circle_fit_temp = circle_fitter_.fitCircle(cluster);
        // 分别根据pca信息和拟合圆信息判别是直线，还是圆弧，以及噪声
        // 噪声检测基本失败
        auto object_type = pca_classifier_.classifyObject(pca_features);

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
      // 圆拟合基本失败,修正圆拟合方法
      // auto circle_fit = circle_fitter_.fitCircle(processed_cluster);
      auto circle_fit = circle_fitter_.fitArcWithRANSAC(processed_cluster);
      if (!circle_fit.is_valid) {
        RCLCPP_WARN(this->get_logger(), "簇 %zu: RANSAC圆拟合失败", idx);
        continue;
      }
      RCLCPP_INFO(this->get_logger(),
                  "簇 %zu : 中心(%.3f,%.3f), 直径=%.3fm, 总误差=%.6f, "
                  "内点数=%d, 内点比例=%.3f, 总点数=%d, 内点误差=%6f, "
                  "外点误差=%6f,  凹半圆检测比率=%4f, 凸半圆检测比率=%4f ",
                  idx, circle_fit.center.x, circle_fit.center.y,
                  circle_fit.radius * 2.0, circle_fit.fit_error,
                  circle_fit.inlier_count, circle_fit.inlier_ratio,
                  circle_fit.total_points, circle_fit.inner_error,
                  circle_fit.outline_error, circle_fit.concave_ratio,
                  circle_fit.convex_ratio);
      if (circle_fit.concave_ratio > 0.2) {
        RCLCPP_WARN(this->get_logger(), "簇 %zu: RANSAC圆拟合为凹型,判定失效",
                    idx);
        continue;
      }
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
   * @brief
   * 计算检测到的Reflectors与跟踪器输出的Confirm之间的结果，来给出确认的检测值
   */
  std::vector<DetectedReflector> confirmDetetedReflectors(
      const std::vector<DetectedReflector> &reflectors,
      const std::vector<TrackedReflector> &confirmed_local_reflectors) {
    std::vector<DetectedReflector> confirmed_reflectors;
    // 计算匹配距离
    // 直接利用确认后的反光柱位置进行匹配
    for (int i = 0; i < reflectors.size(); i++) {
      for (int j = 0; j < confirmed_local_reflectors.size(); j++) {
        if (reflectors[i].center.distanceTo(
                confirmed_local_reflectors[j].global_position) <
            (confirmed_local_reflectors[j].position_std_dev * 2)) {
          confirmed_reflectors.push_back(reflectors[i]);
        }
      }
    }
    return confirmed_reflectors;
  }

  /**
   * @brief Timer callback for continuous publishing
   */
  void publishTimerCallback() {
    if (current_frame_index_ >= frames_.size()) {
      return;
    }
    const auto &frame = frames_[current_frame_index_];
    visualization_helper_->pulishOriginLaserScan(frame->scan);
    // 指定发布frame是以laser为准，还是以矫正后map为准的global_points
    visualization_helper_->publishFilteredPointCloud(frame->compensated_points,
                                                     frame->global_pose);
    // 发布阈值滤波后的点云
    // 指定发布frame是以laser为准，还是以矫正后map为准的global_points
    visualization_helper_->publishClusteredPointCloud(frame->filtered_points,
                                                      frame->global_pose);
    // 新增可视化拟合圆, 默认是以laser为准，可原则是否以map为frame
    visualization_helper_->publishReflectorMarkers(frame->reflectors,
                                                   frame->global_pose);
    // 新增可视化跟踪后的反光柱, 默认是以laser为准，可原则是否以map为frame
    visualization_helper_->publishTrackedReflectorMarkers(
        global_reflector_tracker_.getAllTrackedReflectors(),
        frame->global_pose);
    visualization_helper_->publishLaserPose(frame->global_pose);
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
        // 只考虑{N-1}帧由于双拍缓存，最帧{0,1,...N-2}序列
        if (current_frame_index_ < frames_.size() - 2) {
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

  static std::vector<TimePoint> convertScanToTimedPoints(
      const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<TimePoint> points;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      if (scan_msg->ranges[i] < scan_msg->range_min ||
          scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i])) {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      TimePoint point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      int64_t point_stamp = rclcpp::Time(scan_msg->header.stamp).nanoseconds() +
                            int(i * scan_msg->time_increment * 1e9);
      point.timestamp = point_stamp;
      if (i < scan_msg->intensities.size()) {
        point.intensity = scan_msg->intensities[i];
      } else {
        point.intensity = 0.0;
      }

      points.push_back(point);
    }

    return points;
  }

  static std::vector<Point>
  convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<Point> points;
    int index = 0;
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
      point.origin_index = index++;
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

  void configureDistortionCorrector() {
    this->declare_parameter("distort_corrector.enable", true);
    this->use_distort_corrector_ =
        this->get_parameter("distort_corrector.enable").as_bool();
    this->declare_parameter("distort_corrector.method", "Spline");
    this->distortcorrect_method_ =
        this->get_parameter("distort_corrector.method").as_string();
    if (use_distort_corrector_) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "[✔] 配置点云矫正器: " << distortcorrect_method_);
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "[✘] 没有使用点云矫正器！！！！");
    }
  }

  void configureFixedDBSCAN() {
    FixedDBSCAN::Config dbscan_config;
    this->declare_parameter("fixed_dbscan.eps", 0.05);
    this->declare_parameter("fixed_dbscan.min_points", 5);
    this->declare_parameter("fixed_dbscan.contine_gap", 3);
    this->declare_parameter("fixed_dbscan.continue_points", 8);
    // 提取参数
    dbscan_config.eps = this->get_parameter("fixed_dbscan.eps").as_double();
    dbscan_config.min_points =
        this->get_parameter("fixed_dbscan.min_points").as_int();
    dbscan_config.gap_threshold =
        this->get_parameter("fixed_dbscan.contine_gap").as_int();
    dbscan_config.continue_points =
        this->get_parameter("fixed_dbscan.continue_points").as_int();
    this->fixed_dbscan_ = FixedDBSCAN(dbscan_config);
    // 打印配置信息
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[✔] 配置FixedDBSCAN.eps: " << dbscan_config.eps);
    RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.min_points: "
                                               << dbscan_config.min_points);
    RCLCPP_INFO_STREAM(this->get_logger(), "[✔] 配置FixedDBSCAN.contine_gap: "
                                               << dbscan_config.gap_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[✔] 配置FixedDBSCAN.continue_points: "
                           << dbscan_config.continue_points);
  }

  // Parameters
  std::string bag_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string classification_method_;
  double raw_intensity_threshold_;
  bool enable_interpolation_;
  // 点云矫正器
  bool use_distort_corrector_;
  std::string distortcorrect_method_;
  // 聚类器
  FixedDBSCAN fixed_dbscan_;

  double min_confidence_;
  // 激光与里程计相关数据
  std::unordered_map<int64_t, sensor_msgs::msg::LaserScan::SharedPtr>
      scan_map_; // Map of scan messages by timestamp
  std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr> odom_map_;
  std::vector<int64_t> odom_timestamps_; // Vector of frames
  TimeOrderQueue<transforms::Rigid3d> odom_queue_;
  std::vector<int64_t> scan_timestamps_;

  // Frame data
  std::vector<std::shared_ptr<FrameData>> frames_;
  size_t current_frame_index_;

  // Transform
  geometry_msgs::msg::TransformStamped laser_to_base_;
  // Pose tracking
  PoseTracker pose_tracker_;
  // Detection modules
  PCAShapeClassifier pca_classifier_;
  CircleFitter circle_fitter_;
  PracticalDescriptorExtractor descriptor_extractor_;
  // Global reflector tracking
  GlobalReflectorTracker global_reflector_tracker_;

  // 可视化 Publishers
  std::shared_ptr<VisualizationHelper> visualization_helper_;

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