/*
Author: LeonZack055 (Gmail)
reflector_noise_node.cpp (c) 2026
Desc: Online reflector detection node using ROS2 topic subscriptions
@copyright Copyright (c)  <author> All rights reserved.
@license BSD 2-Clause License
Created:  2026-02-06T06:06:01.815Z
Modified: !date!
*/

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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

#include "amr_reflector_noise_handling/circle_fitter.hpp"
#include "amr_reflector_noise_handling/common/time_order_queue.hpp"
#include "amr_reflector_noise_handling/distort_corrector.hpp"
#include "amr_reflector_noise_handling/fixed_dbscan.hpp"
#include "amr_reflector_noise_handling/global_reflector_tracker.hpp"
#include "amr_reflector_noise_handling/pca_shape_classifier.hpp"
#include "amr_reflector_noise_handling/reflector_detector.hpp"
#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/msg_conversion.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"
#include "amr_reflector_noise_handling/visualization_helper.hpp"

using TimeLaserScan = amr_reflector_noise_handling::TimestampedData<
    sensor_msgs::msg::LaserScan::SharedPtr>;
using namespace amr_reflector_noise_handling;


/**
 * @brief Frame data structure for storing scan and odometry information
 */
struct FrameData {
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  int64_t timestamp; // nanoseconds
  std::vector<TimeRigid3d> between_odoms;
  size_t frame_index;

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
 * @brief Online reflector detection node
 */
class ReflectorNoiseNode : public rclcpp::Node {
public:
  ReflectorNoiseNode()
      : Node("reflector_noise_node"), frame_index_(0),base_frame_("") {
    // Declare parameters
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("odom_topic", "/odom_combined");
    this->declare_parameter("classification_method", "pca");
    this->declare_parameter("raw_intensity_threshold", 1000.0);
    this->declare_parameter("min_confidence", 0.5);
    this->declare_parameter("max_odom_age", 1.0); // Maximum age of odometry data in seconds
    
    // Get parameters
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    classification_method_ =
        this->get_parameter("classification_method").as_string();
    raw_intensity_threshold_ =
        this->get_parameter("raw_intensity_threshold").as_double();
    min_confidence_ = this->get_parameter("min_confidence").as_double();
    max_odom_age_ = this->get_parameter("max_odom_age").as_double();

    // 配置点云矫正器
    configureDistortionCorrector();
    // 配置聚类器
    configureFixedDBSCAN();
    // Configure detection modules
    configureDetectionModules();

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&ReflectorNoiseNode::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 100,
        std::bind(&ReflectorNoiseNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "在线反光柱检测节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "扫描话题: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "里程计话题: %s", odom_topic_.c_str());
  }

  void creaeteVisualizePublisher() {
    // 创建反光柱可视化器
    visualization_helper_ =
        std::make_shared<VisualizationHelper>(shared_from_this());
  }

private:
  /**
   * @brief Callback for laser scan messages
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    int64_t scan_timestamp = rclcpp::Time(scan_msg->header.stamp).nanoseconds();
    
    // Store the scan in the queue
    scan_queue_.push(TimeLaserScan(scan_msg, scan_timestamp));
    // Clean up old scans (keep last 5 seconds)
    while(scan_queue_.size() > 2) {
      auto timeLaserScanPtr = scan_queue_.pop_front();
      processFrame(timeLaserScanPtr);
    }
    // RCLCPP_INFO(this->get_logger(), "Finshi once scan callback!");
  }

  /**
   * @brief Process the oldest frame in the queue (double-frame extraction)
   */
  void processFrame(const TimeLaserScan& current_frame_data) {
    if(odom_queue_.empty()) return;
    // Get the oldest frame and the next frame
    const auto &next_frame_data = scan_queue_.front();
    int64_t scan_timestamp = current_frame_data.timestamp;
    int64_t next_scan_timestamp = next_frame_data.timestamp;
    
    // Try to get the laser_to_base transform from TF
    std::string target_frame = current_frame_data.data->header.frame_id;
    std::string source_frame = base_frame_;

    try {
      // Get transform from base_link to laser frame
      if (!has_laser_to_base_) {
        laser_to_base_ = tf_buffer_->lookupTransform(
            source_frame, target_frame, rclcpp::Time(scan_timestamp),
            std::chrono::milliseconds(100));
        has_laser_to_base_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Finish LaserToBase transform configure!");
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "无法获取从%s到%s的变换: %s",
                          source_frame.c_str(), target_frame.c_str(), ex.what());
      has_laser_to_base_ = false;
      return;
    }

    // Get odometry before current scan
    auto before_vec = odom_queue_.popBefore(scan_timestamp);
    // Get odometry between current scan and next scan
    auto after_vec = odom_queue_.getRange(scan_timestamp, next_scan_timestamp);
    if (before_vec.empty() && after_vec.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[✘] 没有可用的里程计数据 skip current frame");
      return;
    }

    // Create frame data
    auto frame = std::make_shared<FrameData>();
    frame->scan = current_frame_data.data;
    frame->timestamp = scan_timestamp;
    frame->frame_index = frame_index_++;

    // Combine odometry data (double-frame extraction)
    for (const auto &odom : before_vec) {
      frame->between_odoms.emplace_back(odom);
    }
    for (const auto &odom : after_vec) {
      frame->between_odoms.emplace_back(odom);
    }
    // Process the frame
    processFrame(frame);
    // Publish visualization
    publishVisualization(frame);
  }

  /**
   * @brief Callback for odometry messages
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    int64_t timestamp = rclcpp::Time(odom_msg->header.stamp).nanoseconds();
    if (base_frame_ == "") {
      base_frame_ = odom_msg->child_frame_id;
    }
    // Add odometry to the time-ordered queue
    odom_queue_.push(
        TimeRigid3d(transforms::ToRigid3d(odom_msg->pose.pose), timestamp));
    // RCLCPP_INFO(this->get_logger(), "Finshi once odom callback!");
  }

  /**
   * @brief Process a single frame of laser scan data
   */
  void processFrame(std::shared_ptr<FrameData> &frame) {
    // 1. 使用扭曲补偿, 并获取扫描时刻插值轨迹
    auto scan_points = convertScanToTimedPoints(frame->scan);
    
    /// 矫正点云并计算laser在矫正拟合器中估计的全局位姿
    if (!frame->between_odoms.empty() && has_laser_to_base_) {
      auto compensated_points = CublicDistortionCorrector::correctDistortion(
          scan_points, frame->between_odoms,
          transforms::ToRigid3d(laser_to_base_), frame->timestamp,
          frame->global_pose);
      
      if (!use_distort_corrector_) {
        frame->compensated_points = convertScanToPoints(frame->scan);
      } else {
        frame->compensated_points = compensated_points;
      }
    } else {
      // No odometry available or no transform, use raw scan
      frame->compensated_points = convertScanToPoints(frame->scan);
      frame->global_pose = transforms::Rigid3d::Identity();
    }

    // 2. Apply intensity filtering
    auto filtered_points =
        filterByIntensity(frame->compensated_points, raw_intensity_threshold_);
    
    RCLCPP_DEBUG(this->get_logger(), "处理帧 %zu, 原始点数: %zu, 强度过滤后: %zu",
                 frame->frame_index, frame->compensated_points.size(), filtered_points.size());

    // 3. DBSCAN clustering
    frame->filtered_points = filtered_points;
    std::vector<std::vector<Point>> clusters =
        fixed_dbscan_.splitCluster(frame->filtered_points);
    
    if (clusters.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "DBSCAN聚类后无簇");
      frame->reflectors.clear();
      return;
    }

    // 4. Detect reflectors
    if (use_short_tracker_) {
      frame->reflectors = reflector_detector_.detectReflectorsWithShortTracking(
          clusters, frame->global_pose, frame->timestamp);
      RCLCPP_DEBUG(this->get_logger(), "短时跟踪检测到 %zu 个反光柱",
                   frame->reflectors.size());
    } else {
      frame->reflectors = reflector_detector_.detectReflectors(clusters);
      RCLCPP_DEBUG(this->get_logger(), "检测到 %zu 个反光柱",
                   frame->reflectors.size());
    }
  }

  /**
   * @brief Publish visualization data
   */
  void publishVisualization(std::shared_ptr<FrameData> &frame) {
    visualization_helper_->pulishOriginLaserScan(frame->scan);
    visualization_helper_->publishFilteredPointCloud(frame->compensated_points,
                                                     frame->global_pose);
    visualization_helper_->publishClusteredPointCloud(frame->filtered_points,
                                                      frame->global_pose);
    visualization_helper_->publishReflectorMarkers(frame->reflectors,
                                                   frame->global_pose);
    visualization_helper_->publishTrackedReflectorMarkers(
        reflector_detector_.getCurrentAllTrackedReflectors(),
        frame->global_pose);
    visualization_helper_->publishLaserPose(frame->global_pose);
  }

  /*
   * @brief 将LaserScan转化成带有时间戳，强度和有向序列的二维点
   */
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

  void configureDistortionCorrector() {
    this->declare_parameter("distort_corrector.enable", true);
    this->declare_parameter("distort_corrector.method", "Spline");
    this->use_distort_corrector_ =
        this->get_parameter("distort_corrector.enable").as_bool();
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
    
    dbscan_config.eps = this->get_parameter("fixed_dbscan.eps").as_double();
    dbscan_config.min_points =
        this->get_parameter("fixed_dbscan.min_points").as_int();
    dbscan_config.gap_threshold =
        this->get_parameter("fixed_dbscan.contine_gap").as_int();
    dbscan_config.continue_points =
        this->get_parameter("fixed_dbscan.continue_points").as_int();
    
    this->fixed_dbscan_ = FixedDBSCAN(dbscan_config);
    
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

  void configureDetectionModules() {
    configurePCAClassification();
    configureCicrleFit();
    configureGlobalTracker();
  }

  void configurePCAClassification() {
    this->declare_parameter("pca_classification.enable", true);
    this->declare_parameter("pca_classification.min_points", 13);
    this->declare_parameter("pca_classification.max_elongation_post", 9.5);
    this->declare_parameter("pca_classification.min_elongation_board", 12.0);
    this->declare_parameter("pca_classification.max_linearity_post", 0.97);
    this->declare_parameter("pca_classification.min_linearity_board", 0.93);
    this->declare_parameter("pca_classification.near_distance", 1.3);
    this->declare_parameter("pca_classification.near_min_points", 20);
    this->declare_parameter("pca_classification.near_max_linearity_post", 0.89);
    
    bool use_pca_classification =
        this->get_parameter("pca_classification.enable").as_bool();
    ShapeClassificationParams pca_params;
    pca_params.min_points =
        this->get_parameter("pca_classification.min_points").as_int();
    pca_params.max_elongation_post =
        this->get_parameter("pca_classification.max_elongation_post")
            .as_double();
    pca_params.min_elongation_board =
        this->get_parameter("pca_classification.min_elongation_board")
            .as_double();
    pca_params.max_linearity_post =
        this->get_parameter("pca_classification.max_linearity_post")
            .as_double();
    pca_params.min_linearity_board =
        this->get_parameter("pca_classification.min_linearity_board")
            .as_double();
    pca_params.near_distance =
        this->get_parameter("pca_classification.near_distance").as_double();
    pca_params.near_min_points =
        this->get_parameter("pca_classification.near_min_points").as_int();
    pca_params.near_max_linearity_post =
        this->get_parameter("pca_classification.near_max_linearity_post")
            .as_double();
    
    reflector_detector_.setDetectMethod(classification_method_);
    if (use_pca_classification) {
      reflector_detector_.configPCAShapeClassifier(pca_params);
    }
  }

  void configureCicrleFit() {
    this->declare_parameter("circle_fit.max_fit_error", 0.03);
    this->declare_parameter("circle_fit.min_inlier_ratio", 0.5);
    this->declare_parameter("circle_fit.max_fit_error_near", 0.01);
    this->declare_parameter("circle_fit.max_fit_error_far", 0.02);
    this->declare_parameter("circle_fit.far_distance_threshold", 1.5);
    this->declare_parameter("circle_fit.min_radius", 0.02);
    this->declare_parameter("circle_fit.max_radius", 0.05);
    this->declare_parameter("circle_fit.max_concave_ratio", 0.2);
    this->declare_parameter("circle_fit.ransac_iterations", 100);
    this->declare_parameter("circle_fit.ransac_inlier_threshold", 0.0015);
    this->declare_parameter("circle_fit.ransac_min_points", 13);
    
    CircleFitParams circle_params;
    circle_params.max_fit_error =
        this->get_parameter("circle_fit.max_fit_error").as_double();
    circle_params.min_inlier_ratio =
        this->get_parameter("circle_fit.min_inlier_ratio").as_double();
    circle_params.max_fit_error_near =
        this->get_parameter("circle_fit.max_fit_error_near").as_double();
    circle_params.max_fit_error_far =
        this->get_parameter("circle_fit.max_fit_error_far").as_double();
    circle_params.far_distance_threshold =
        this->get_parameter("circle_fit.far_distance_threshold").as_double();
    circle_params.min_radius =
        this->get_parameter("circle_fit.min_radius").as_double();
    circle_params.max_radius =
        this->get_parameter("circle_fit.max_radius").as_double();
    circle_params.max_concave_ratio =
        this->get_parameter("circle_fit.max_concave_ratio").as_double();
    circle_params.ransac_iterations =
        this->get_parameter("circle_fit.ransac_iterations").as_int();
    circle_params.ransac_inlier_threshold =
        this->get_parameter("circle_fit.ransac_inlier_threshold").as_double();
    circle_params.ransac_min_points =
        this->get_parameter("circle_fit.ransac_min_points").as_int();
    
    reflector_detector_.configCircleFitter(circle_params);
  }

  void configureGlobalTracker() {
    this->declare_parameter("global_tracking.enable", true);
    this->declare_parameter("global_tracking.match_distance_threshold", 0.3);
    this->declare_parameter("global_tracking.match_distance_inactive", 0.5);
    this->declare_parameter("global_tracking.confirm_time_window", 1.0);
    this->declare_parameter("global_tracking.min_detections_in_window", 8);
    this->declare_parameter("global_tracking.inactive_timeout", 5.0);
    this->declare_parameter("global_tracking.max_inactive_time", 60.0);
    this->declare_parameter("global_tracking.position_filter_alpha", 0.3);
    this->declare_parameter("global_tracking.position_filter_beta", 0.2);
    this->declare_parameter("global_tracking.min_std_dev", 0.02);
    this->declare_parameter("global_tracking.max_std_dev", 0.5);
    this->declare_parameter("global_tracking.min_confidence_to_track", 0.3);
    this->declare_parameter("global_tracking.confidence_filter_alpha", 0.2);
    this->declare_parameter("global_tracking.diameter_filter_alpha", 0.3);

    bool use_global_tracker =
        this->get_parameter("global_tracking.enable").as_bool();
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
    
    if (use_global_tracker) {
      RCLCPP_INFO(
          this->get_logger(),
          "#===反光柱检测器==== 开启全局反光柱跟踪器短时跟踪功能!!! ###");
      reflector_detector_.configGlobalReflectorTracker(tracking_config);
    }
    use_short_tracker_ = use_global_tracker;
  }

  //---------------------- Parameters -------------------
  std::string scan_topic_;
  std::string odom_topic_;
  std::string classification_method_;
  double raw_intensity_threshold_;
  double min_confidence_;
  double max_odom_age_; // Maximum age of odometry data for interpolation

  // TF transform
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped laser_to_base_;
  std::string base_frame_;
  bool has_laser_to_base_{false};

  // ROS2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // LaserScan queue for double-frame extraction
  TimeOrderQueue<sensor_msgs::msg::LaserScan::SharedPtr> scan_queue_;
  size_t frame_index_;

  // Odometry queue for time-based interpolation
  TimeOrderQueue<transforms::Rigid3d> odom_queue_;

  // Point cloud corrector
  bool use_distort_corrector_;
  std::string distortcorrect_method_;

  // Clustering
  FixedDBSCAN fixed_dbscan_;

  // Detection modules
  bool use_short_tracker_;
  RefelctorDetector reflector_detector_;

  // Visualization
  std::shared_ptr<VisualizationHelper> visualization_helper_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ReflectorNoiseNode>();
  node->creaeteVisualizePublisher();

  RCLCPP_INFO(node->get_logger(), "启动在线反光柱检测节点");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}