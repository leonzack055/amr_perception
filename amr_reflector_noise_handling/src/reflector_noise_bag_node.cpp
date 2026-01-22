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

using namespace amr_reflector_noise_handling;

/**
 * @brief Frame data structure for storing scan and odometry information
 */
struct FrameData
{
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  nav_msgs::msg::Odometry::SharedPtr odom;
  int64_t timestamp;  // nanoseconds
  std::vector<int64_t> between_next_odoms;
  size_t odom_count;
  size_t frame_index;  // laserscan的索引

  // Compensated point cloud (after distortion correction)
  std::vector<Point> compensated_points;

  // Detected reflectors
  std::vector<DetectedReflector> reflectors;

  // Global pose in world frame
  geometry_msgs::msg::Pose global_pose;

  FrameData() : frame_index(0)
  {
  }
};

/**
 * @brief Global pose tracking using odometry
 */
class PoseTracker
{
public:
  PoseTracker() : initialized_(false)
  {
  }

  /**
   * @brief Update global pose using odometry
   */
  void update(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    if (!initialized_)
    {
      // Initialize with first odometry
      global_pose_ = odom->pose.pose;
      initial_pose_ = odom->pose.pose;
      initialized_ = true;
      return;
    }

    // Compute relative transform from previous to current odometry
    geometry_msgs::msg::Transform relative_transform = computeRelativeTransform(prev_odom_, odom);

    // Apply relative transform to global pose
    applyTransform(global_pose_, relative_transform);

    prev_odom_ = odom;
  }

  /**
   * @brief Get current global pose
   */
  geometry_msgs::msg::Pose getGlobalPose() const
  {
    return global_pose_;
  }

  /**
   * @brief Reset tracker
   */
  void reset()
  {
    initialized_ = false;
    global_pose_ = geometry_msgs::msg::Pose();
    initial_pose_ = geometry_msgs::msg::Pose();
    prev_odom_ = nullptr;
  }

  /**
   * @brief Get trajectory points
   */
  const std::vector<geometry_msgs::msg::PoseStamped>& getTrajectory() const
  {
    return trajectory_;
  }

  /**
   * @brief Add trajectory point
   */
  void addTrajectoryPoint(const geometry_msgs::msg::Pose& pose, const rclcpp::Time& time)
  {
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
  geometry_msgs::msg::Transform computeRelativeTransform(const nav_msgs::msg::Odometry::SharedPtr odom1,
                                                         const nav_msgs::msg::Odometry::SharedPtr odom2)
  {
    geometry_msgs::msg::Transform transform;

    // Compute relative translation
    transform.translation.x = odom2->pose.pose.position.x - odom1->pose.pose.position.x;
    transform.translation.y = odom2->pose.pose.position.y - odom1->pose.pose.position.y;
    transform.translation.z = odom2->pose.pose.position.z - odom1->pose.pose.position.z;

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
  void applyTransform(geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Transform& transform)
  {
    // Apply rotation
    tf2::Quaternion q_pose, q_transform;
    tf2::fromMsg(pose.orientation, q_pose);
    tf2::fromMsg(transform.rotation, q_transform);
    tf2::Quaternion q_new = q_pose * q_transform;
    pose.orientation = tf2::toMsg(q_new);

    // Apply translation (rotated into new frame)
    tf2::Vector3 trans(transform.translation.x, transform.translation.y, transform.translation.z);
    tf2::Vector3 rotated_trans = tf2::quatRotate(q_pose, trans);
    pose.position.x += rotated_trans.x();
    pose.position.y += rotated_trans.y();
    pose.position.z += rotated_trans.z();
  }
};

/**
 * @brief Point cloud distortion correction using odometry
 */
class DistortionCorrector
{
public:
  /**
   * @brief Correct point cloud distortion using odometry between frames
   */
  static std::vector<Point> correctDistortion(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                              const nav_msgs::msg::Odometry::SharedPtr odom_start,
                                              const nav_msgs::msg::Odometry::SharedPtr odom_end)
  {
    if (!odom_start || !odom_end)
    {
      // No odometry available, return original points
      return convertScanToPoints(scan);
    }

    std::vector<Point> corrected_points;
    corrected_points.reserve(scan->ranges.size());

    // Convert scan to points first
    auto original_points = convertScanToPoints(scan);

    // Interpolate odometry for each scan point
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max || !std::isfinite(scan->ranges[i]))
      {
        continue;
      }

      // Compute scan time for this point
      double scan_time = scan->scan_time;
      double point_time = scan->time_increment * i;
      double alpha = point_time / scan_time;  // 0 at start, 1 at end

      // Interpolate odometry
      auto interpolated_odom = interpolateOdometry(odom_start, odom_end, alpha);

      // Transform point to world frame using interpolated odometry
      Point corrected = transformPointToWorld(original_points[i], interpolated_odom);
      corrected_points.push_back(corrected);
    }

    return corrected_points;
  }

private:
  /**
   * @brief Convert laser scan to points
   */
  static std::vector<Point> convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    std::vector<Point> points;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
      if (scan_msg->ranges[i] < scan_msg->range_min || scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i]))
      {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      Point point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);

      if (i < scan_msg->intensities.size())
      {
        point.intensity = scan_msg->intensities[i];
      }
      else
      {
        point.intensity = 0.0;
      }

      points.push_back(point);
    }

    return points;
  }

  /**
   * @brief Interpolate odometry between two poses
   */
  static geometry_msgs::msg::Pose interpolateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom1,
                                                      const nav_msgs::msg::Odometry::SharedPtr odom2, double alpha)
  {
    geometry_msgs::msg::Pose interpolated;

    // Interpolate position
    interpolated.position.x =
        odom1->pose.pose.position.x + alpha * (odom2->pose.pose.position.x - odom1->pose.pose.position.x);
    interpolated.position.y =
        odom1->pose.pose.position.y + alpha * (odom2->pose.pose.position.y - odom1->pose.pose.position.y);
    interpolated.position.z =
        odom1->pose.pose.position.z + alpha * (odom2->pose.pose.position.z - odom1->pose.pose.position.z);

    // Interpolate orientation using SLERP
    tf2::Quaternion q1, q2;
    tf2::fromMsg(odom1->pose.pose.orientation, q1);
    tf2::fromMsg(odom2->pose.pose.orientation, q2);
    tf2::Quaternion q_interp = q1.slerp(q2, alpha);
    interpolated.orientation = tf2::toMsg(q_interp);

    return interpolated;
  }

  /**
   * @brief Transform point from laser frame to world frame
   */
  static Point transformPointToWorld(const Point& point, const geometry_msgs::msg::Pose& odom_pose)
  {
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
class ReflectorNoiseBagNode : public rclcpp::Node
{
public:
  ReflectorNoiseBagNode()
    : Node("reflector_noise_bag_node"), current_frame_index_(0), auto_mode_(false), should_exit_(false)
  {
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
    classification_method_ = this->get_parameter("classification_method").as_string();
    raw_intensity_threshold_ = this->get_parameter("raw_intensity_threshold").as_double();
    expected_diameter_ = this->get_parameter("expected_diameter").as_double();
    diameter_tolerance_ = this->get_parameter("diameter_tolerance").as_double();
    enable_interpolation_ = this->get_parameter("enable_interpolation").as_bool();
    min_confidence_ = this->get_parameter("min_confidence").as_double();

    // Configure detection modules
    configureDetectionModules();

    // Create publisher for visualization
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reflector_cluster_cloud", 10);
    compensated_cluster_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/reflector_compensated_cluster_cloud", 10);
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/reflector_bag_trajectory", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/reflector_detected_markers", 10);
    tracked_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/reflector_tracked_markers", 10);

    // Create timer for continuous publishing (10 Hz)
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&ReflectorNoiseBagNode::publishTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "反光柱逐帧检测节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "Bag路径: %s", bag_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "扫描话题: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "里程计话题: %s", odom_topic_.c_str());
  }

  /**
   * @brief Run the bag processing
   */
  void run()
  {
    if (bag_path_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Bag路径未设置，请使用--ros-args -p bag_path:=<path>");
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
    for (auto& topic : reader.get_all_topics_and_types())
    {
      RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s", topic.name.c_str(), topic.type.c_str());
    }

    // Pre-load all frames
    if (!loadAllFrames(reader))
    {
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
    input_thread.join();

    RCLCPP_INFO(this->get_logger(), "处理完成");
  }

private:
  /**
   * @brief Configure detection modules
   */
  void configureDetectionModules()
  {
    // Configure circle fitter
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
    pca_params.min_linearity_for_board = 0.94;
    pca_params.max_linearity_for_post = 0.978;
    pca_params.min_circularity_for_post = 0.64;
    pca_params.min_circularity_for_board = 0.46;
    pca_classifier_.setParams(pca_params);

    // Configure geometric validator
    geometric_validator_.setExpectedDiameter(expected_diameter_);
    geometric_validator_.setDiameterTolerance(diameter_tolerance_);
  }

  /**
   * @brief Load all frames from bag
   */
  bool loadAllFrames(rosbag2_cpp::Reader& reader)
  {
    std::unordered_map<int64_t, sensor_msgs::msg::LaserScan::SharedPtr> scan_map;  // Map of scan messages by timestamp
    std::unordered_map<int64_t, nav_msgs::msg::Odometry::SharedPtr> odom_map;
    std::vector<int64_t> odom_timestamps;  // Vector of frames
    std::vector<int64_t> scan_timestamps;
    int64_t peek_time = 0;  // Peek time for next message

    auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
    // auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    auto tf_serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();

    // Read all messages
    RCLCPP_INFO(this->get_logger(), "开始读取rosbag包: %s ..... ", bag_path_.c_str());
    while (reader.has_next())
    {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (msg->time_stamp > peek_time)
      {
        peek_time = msg->time_stamp;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "录制的rosbag包出现前后事件跳变: 消息 %s, %ld", msg->topic_name.c_str(),
                     msg->time_stamp);
      }
      // Process based on topic
      if (msg->topic_name == scan_topic_)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "读取scan消息: ");
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
        laser_scan_serializer.deserialize_message(&serialized_msg, scan.get());
        if (scan_map.find(rclcpp::Time(msg->time_stamp).nanoseconds()) == scan_map.end())
        {
          scan_map[rclcpp::Time(msg->time_stamp).nanoseconds()] = scan;
          scan_timestamps.push_back(rclcpp::Time(msg->time_stamp).nanoseconds());
        }
        else
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "读取scan消息: "
                                                      << " bag timestamp: " << msg->time_stamp << "存在重复时间戳");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "完成读取scan消息: ");
      }
      else if (msg->topic_name == odom_topic_)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "读取odom消息: ");
        auto odom = std::make_shared<nav_msgs::msg::Odometry>();
        odom_serializer.deserialize_message(&serialized_msg, odom.get());
        if (odom_map.find(rclcpp::Time(msg->time_stamp).nanoseconds()) == odom_map.end())
        {
          odom_map[rclcpp::Time(msg->time_stamp).nanoseconds()] = odom;
          odom_timestamps.push_back(rclcpp::Time(msg->time_stamp).nanoseconds());
        }
        else
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "读取odom消息: "
                                                      << " bag timestamp: " << msg->time_stamp << "存在重复时间戳");
        }
        odom_map[rclcpp::Time(msg->time_stamp).nanoseconds()] = odom;
        odom_timestamps.push_back(rclcpp::Time(msg->time_stamp).nanoseconds());
        RCLCPP_INFO_STREAM(this->get_logger(), "完成读取odom消息: ");
      }
      else if (msg->topic_name == "/tf_static")
      {
        // Read tf_static for laser_scan to base_link transform
        auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        try
        {
          tf_serializer.deserialize_message(&serialized_msg, tf_msg.get());
          for (auto& transform : tf_msg->transforms)
          {
            if (transform.header.frame_id == "base_link" && transform.child_frame_id == "laser")
            {
              laser_to_base_ = transform;
              RCLCPP_INFO(this->get_logger(), "找到laser到base_link的变换: (%.3f, %.3f)",
                          transform.transform.translation.x, transform.transform.translation.y);
            }
          }
        }
        catch (const rclcpp::exceptions::RCLError& rcl_error)
        {
          RCLCPP_ERROR_STREAM(this->get_logger(), "解析TF_STATIC发生错误" << rcl_error.what());
        }
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "完成读取rosbag包:  ..... "<< bag_path_);
    // 展示整体队列和信息内容:
    RCLCPP_INFO_STREAM(this->get_logger(), "激光雷达队列信息: " << scan_map.size() << "帧, 起始范围: ["
                                                                << scan_timestamps.front() << " , "
                                                                << scan_timestamps.back() << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "里程计队列信息: " << odom_map.size() << "帧, 起始范围: ["
                                                              << scan_timestamps.front() << " , "
                                                              << scan_timestamps.back() << "]");
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
    std::vector<int64_t>::const_iterator odom_peek = odom_timestamps.cbegin();
    std::sort(scan_timestamps.begin(), scan_timestamps.end());
    std::sort(odom_timestamps.begin(), odom_timestamps.end());
    for (std::vector<int64_t>::const_iterator scan_iterator = scan_timestamps.cbegin();
         scan_iterator < scan_timestamps.cend() - 1; ++scan_iterator)
    {
      const auto& next_scan_iterator = scan_iterator + 1;
      const auto& scan_time = *scan_iterator;
      const auto& next_scan_time = *next_scan_iterator;
      auto frame = std::make_shared<FrameData>();
      frame->scan = scan_map[scan_time];
      frame->timestamp = scan_time;
      frame->frame_index = frame_idx++;
      // Find closest odometry
      std::vector<int64_t>::const_iterator keep_odom_peek = odom_peek;
      for (; odom_peek < odom_timestamps.cend(); ++odom_peek)
      {
        if (*odom_peek <= scan_time)
        {
          frame->between_next_odoms.push_back(*odom_peek);
          keep_odom_peek++;  // 保留当前的odom_peek，用于下一次迭代，查找相邻帧的数据
        }
        if (*odom_peek > scan_time && *odom_peek < next_scan_time)
        {
          frame->between_next_odoms.push_back(*odom_peek);
        }
      }
      odom_peek = keep_odom_peek;
      // 记录数据
      frames_.push_back(frame);
    }
    RCLCPP_INFO(this->get_logger(), "激光雷达数据为: %d帧", frames_.size());
    // 补偿最后一帧激光雷达的odom数据虽然它可能只有一半
    if (odom_peek < odom_timestamps.cend())
    {
      for (std::vector<int64_t>::const_iterator scan_iterator = scan_timestamps.cend() - 1;
           scan_iterator < scan_timestamps.cend(); ++scan_iterator)
      {
        const auto& scan_time = *scan_iterator;
        auto frame = std::make_shared<FrameData>();
        frame->scan = scan_map[scan_time];
        frame->timestamp = scan_time;
        frame->frame_index = frame_idx++;
        for (; odom_peek < odom_timestamps.cend(); ++odom_peek)
        {
          if (*odom_peek <= scan_time)
          {
            frame->between_next_odoms.push_back(*odom_peek);
          }
        }
        frames_.push_back(frame);
        RCLCPP_INFO(this->get_logger(), "补偿激光雷达数据为: %d帧, 使用的里程计数据: %ld帧", frames_.size(),
                    frame->between_next_odoms.size());
      }
    }
    return !frames_.empty();
  }

  /**
   * @brief Process a single frame
   */
  void processFrame(size_t frame_index)
  {
    if (frame_index >= frames_.size())
    {
      RCLCPP_WARN(this->get_logger(), "帧索引超出范围: %zu/%zu", frame_index, frames_.size());
      return;
    }

    current_frame_index_ = frame_index;
    auto& frame = frames_[frame_index];

    RCLCPP_INFO(this->get_logger(), "处理帧 %zu/%zu, 时间: %.3f", frame_index, frames_.size() - 1,
                rclcpp::Time(frame->timestamp).seconds());

    // Update global pose tracker
    if (frame->odom)
    {
      pose_tracker_.update(frame->odom);
      frame->global_pose = pose_tracker_.getGlobalPose();
      pose_tracker_.addTrajectoryPoint(frame->global_pose, rclcpp::Time(frame->timestamp));
    }

    // Apply distortion correction
    if (frame_index > 0 && frame->odom && frames_[frame_index - 1]->odom)
    {
      frame->compensated_points =
          DistortionCorrector::correctDistortion(frame->scan, frames_[frame_index - 1]->odom, frame->odom);
      RCLCPP_INFO(this->get_logger(), "应用畸变校正");
    }
    else
    {
      // No previous odometry, convert directly
      frame->compensated_points = convertScanToPoints(frame->scan);
    }

    // Apply intensity filtering
    auto filtered_points = filterByIntensity(frame->compensated_points, raw_intensity_threshold_);

    RCLCPP_INFO(this->get_logger(), "原始点数: %zu, 强度过滤后: %zu", frame->compensated_points.size(),
                filtered_points.size());

    // DBSCAN clustering
    auto cluster_indices = fixed_dbscan_.cluster(filtered_points);

    if (cluster_indices.empty())
    {
      RCLCPP_DEBUG(this->get_logger(), "DBSCAN聚类后无簇");
      frame->reflectors.clear();
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "DBSCAN聚类得到 %zu 个簇", cluster_indices.size());

    // Convert indices to clusters
    std::vector<std::vector<Point>> clusters;
    for (const auto& indices : cluster_indices)
    {
      std::vector<Point> cluster;
      cluster.reserve(indices.size());
      for (int idx : indices)
      {
        cluster.push_back(filtered_points[idx]);
      }
      clusters.push_back(cluster);
    }

    // Detect reflectors
    detectReflectors(clusters, frame->reflectors);

    RCLCPP_INFO(this->get_logger(), "检测到 %zu 个反光柱", frame->reflectors.size());

    // Visualization data is ready, will be published by timer
  }

  /**
   * @brief Detect reflectors from clusters
   */
  void detectReflectors(const std::vector<std::vector<Point>>& clusters, std::vector<DetectedReflector>& reflectors)
  {
    reflectors.clear();

    for (size_t idx = 0; idx < clusters.size(); ++idx)
    {
      const auto& cluster = clusters[idx];
      Point center = computeCentroid(cluster);
      double distance = center.distanceFromOrigin();

      // Classification
      bool is_reflector_candidate = false;

      if (classification_method_ == "pca")
      {
        auto pca_features = pca_classifier_.computeShapeFeatures(cluster, circle_fitter_);
        auto circle_fit_temp = circle_fitter_.fitCircle(cluster);
        auto object_type = pca_classifier_.classifyObject(pca_features, circle_fit_temp);

        is_reflector_candidate = (object_type == REFLECTOR_POST);

        RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 延伸度=%.2f, 线性度=%.2f, 圆形度=%.2f, 类型=%s", idx,
                     pca_features.elongation, pca_features.linearity, pca_features.circularity,
                     is_reflector_candidate ? "反光柱" : "其他");
      }

      if (!is_reflector_candidate)
      {
        continue;
      }

      // Interpolation
      auto processed_cluster = cluster;
      if (enable_interpolation_)
      {
        processed_cluster = improved_interpolator_.interpolateCluster(cluster, center, distance);
      }

      // Circle fitting
      auto circle_fit = circle_fitter_.fitCircle(processed_cluster);

      if (!circle_fitter_.validateFit(circle_fit, cluster.size(), distance))
      {
        continue;
      }

      // Compute confidence
      double confidence = computeConfidence(cluster, circle_fit);

      if (confidence >= min_confidence_)
      {
        DetectedReflector reflector;
        reflector.center = circle_fit.center;
        reflector.diameter = 2 * circle_fit.radius;
        reflector.confidence = confidence;
        reflector.point_count = cluster.size();

        reflectors.push_back(reflector);

        RCLCPP_INFO(this->get_logger(), "反光柱 %zu: 中心(%.3f,%.3f), 直径=%.3fm, 置信度=%.3f", reflectors.size(),
                    reflector.center.x, reflector.center.y, reflector.diameter, confidence);
      }
    }
  }

  /**
   * @brief Compute confidence for a detected reflector
   */
  double computeConfidence([[maybe_unused]] const std::vector<Point>& cluster, const CircleFitResult& circle_fit)
  {
    // Simple confidence based on fit quality
    double fit_quality = 1.0 - std::min(1.0, circle_fit.fit_error / 0.03);
    double inlier_quality = circle_fit.inlier_ratio;

    return 0.6 * fit_quality + 0.4 * inlier_quality;
  }

  /**
   * @brief Timer callback for continuous publishing
   */
  void publishTimerCallback()
  {
    if (current_frame_index_ >= frames_.size())
    {
      return;
    }

    const auto& frame = frames_[current_frame_index_];

    // Publish point cloud with current timestamp
    publishPointCloud(frame->compensated_points);

    // Publish reflector markers with current timestamp
    publishReflectorMarkers(frame->reflectors);

    // Publish trajectory with current timestamp
    publishTrajectory();
  }

  /**
   * @brief Keyboard input thread
   */
  void keyboardInputThread()
  {
    RCLCPP_INFO(this->get_logger(), "键盘控制:");
    RCLCPP_INFO(this->get_logger(), "  'n' - 下一帧");
    RCLCPP_INFO(this->get_logger(), "  'p' - 上一帧");
    RCLCPP_INFO(this->get_logger(), "  ' ' (空格) - 切换自动模式");
    RCLCPP_INFO(this->get_logger(), "  'q' - 退出");

    while (!should_exit_)
    {
      char key = std::getchar();

      switch (key)
      {
        case 'n':
        case 'N':
          if (current_frame_index_ < frames_.size() - 1)
          {
            processFrame(current_frame_index_ + 1);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "已是最后一帧");
          }
          break;

        case 'p':
        case 'P':
          if (current_frame_index_ > 0)
          {
            processFrame(current_frame_index_ - 1);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "已是第一帧");
          }
          break;

        case ' ':
          auto_mode_ = !auto_mode_;
          RCLCPP_INFO(this->get_logger(), "自动模式: %s", auto_mode_ ? "开启" : "关闭");
          if (auto_mode_)
          {
            startAutoMode();
          }
          break;

        case 'q':
        case 'Q':
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
  void startAutoMode()
  {
    std::thread([this]() {
      while (auto_mode_ && !should_exit_)
      {
        if (current_frame_index_ < frames_.size() - 1)
        {
          processFrame(current_frame_index_ + 1);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "自动处理完成");
          auto_mode_ = false;
          break;
        }
      }
    }).detach();
  }

  /**
   * @brief Publish point cloud with current timestamp
   */
  void publishPointCloud(const std::vector<Point>& points)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "base_link";
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

    for (size_t i = 0; i < points.size(); ++i)
    {
      uint8_t* ptr = &cloud_msg.data[i * cloud_msg.point_step];

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

  /**
   * @brief Publish reflector markers with current timestamp
   */
  void publishReflectorMarkers(const std::vector<DetectedReflector>& reflectors)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(reflectors.size());

    for (size_t i = 0; i < reflectors.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = "base_link";
      marker.ns = "reflective_posts";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = reflectors[i].center.x;
      marker.pose.position.y = reflectors[i].center.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = reflectors[i].diameter;
      marker.scale.y = reflectors[i].diameter;
      marker.scale.z = 0.5;

      double confidence = reflectors[i].confidence;
      marker.color.r = 0.0;
      marker.color.g = confidence;
      marker.color.b = 1.0 - confidence;
      marker.color.a = 0.8;

      marker.lifetime = rclcpp::Duration::from_seconds(0.2);

      marker_array.markers[i] = marker;
    }

    marker_pub_->publish(marker_array);
  }

  /**
   * @brief Publish trajectory
   */
  void publishTrajectory()
  {
    const auto& trajectory = pose_tracker_.getTrajectory();
    if (trajectory.empty())
    {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.resize(trajectory.size());
    for (size_t i = 0; i < trajectory.size(); ++i)
    {
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
  std::vector<Point> convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    std::vector<Point> points;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
      if (scan_msg->ranges[i] < scan_msg->range_min || scan_msg->ranges[i] > scan_msg->range_max ||
          !std::isfinite(scan_msg->ranges[i]))
      {
        continue;
      }

      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double range = scan_msg->ranges[i];

      Point point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);

      if (i < scan_msg->intensities.size())
      {
        point.intensity = scan_msg->intensities[i];
      }
      else
      {
        point.intensity = 0.0;
      }

      points.push_back(point);
    }

    return points;
  }

  /**
   * @brief Filter points by intensity
   */
  std::vector<Point> filterByIntensity(const std::vector<Point>& points, double threshold)
  {
    std::vector<Point> filtered;
    filtered.reserve(points.size());

    for (const auto& point : points)
    {
      if (point.intensity >= threshold)
      {
        filtered.push_back(point);
      }
    }

    return filtered;
  }

  /**
   * @brief Compute centroid of cluster
   */
  Point computeCentroid(const std::vector<Point>& cluster)
  {
    Point centroid;
    if (cluster.empty())
    {
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& point : cluster)
    {
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
  CircleFitter circle_fitter_;
  PracticalDescriptorExtractor descriptor_extractor_;
  GeometricValidator geometric_validator_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr compensated_cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;

  // Timer for continuous publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Threading
  std::atomic<bool> auto_mode_;
  std::atomic<bool> should_exit_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReflectorNoiseBagNode>();

  RCLCPP_INFO(node->get_logger(), "启动反光柱逐帧检测 (Bag处理版本)");

  node->run();

  rclcpp::shutdown();

  return 0;
}