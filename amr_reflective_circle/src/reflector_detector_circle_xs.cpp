#include <algorithm>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cartographer_ros_msgs/msg/landmark_entry.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>
#include <unordered_set>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "landmark_localization/common/msg_conversion.hpp"
#include "landmark_localization/landmark_assigner.hpp"
#include "landmark_localization/reflective_post_detector.hpp"

using namespace std::chrono_literals;

// std::ofstream outFile("./test_cpp_circle_r.txt");

class ReflectorDetectorCircle : public rclcpp::Node {
public:
  ReflectorDetectorCircle() : Node("reflector_detector_maker") {
    // 参数声明
    this->declare_parameter("intensity_threshold_use", 1000);
    this->declare_parameter("cluster_eps", 0.065);
    this->declare_parameter("min_cluster_points", 4);
    this->declare_parameter("diameter_min", 0.05);
    this->declare_parameter("diameter_max", 0.12);
    this->declare_parameter("residual_avg_threshold", 0.01);
    this->declare_parameter("residual_std_threshold", 0.005);
    this->declare_parameter("residual_max_threshold", 0.02);
    this->declare_parameter("stat_mean_k", 4);
    this->declare_parameter("stat_std_threshold", 1.0);
    this->declare_parameter("max_history_age", 3);
    this->declare_parameter("match_distance_threshold", 0.2);
    this->declare_parameter("arc_threshold", 0.1);
    this->declare_parameter("max_arc_feature", 30.0);
    this->declare_parameter("arc_min_points", 4);
    this->declare_parameter("residual_real", 0.032);
    this->declare_parameter("sensitivity", 2.0);
    this->declare_parameter("maxError", 1.0);
    this->declare_parameter("maxangleError", 0.1);
    this->declare_parameter("landmark_rotation_weight", 1e2);
    this->declare_parameter("landmark_translation_weight", 1.0);

    // 订阅和发布
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ReflectorDetectorCircle::scan_callback, this,
                  std::placeholders::_1));

    marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reflector_markers", 10);

    RCLCPP_INFO(this->get_logger(), "反光柱检测节点初始化完成");

    // 1. 创建反光柱检测器
    landmark_detector_ =
        std::make_shared<landmark_localization::ReflectivePostDetector>(
            this->get_parameter("intensity_threshold_use").as_int());

    // 3. 创建tf缓存机制
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        this->get_clock(), tf2::durationFromSec(kTfBufferCacheTimeInSeconds));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    init();
  }

  void init() {
    // 获取参数
    landmark_rotation_weight_ =
        this->get_parameter("landmark_rotation_weight").as_double();
    landmark_translation_weight_ =
        this->get_parameter("landmark_translation_weight").as_double();

    // 1. 提取检测器参数
    landmark_detector_->intensity_threshold_use =
        this->get_parameter("intensity_threshold_use").as_int();
    landmark_detector_->arc_threshold =
        this->get_parameter("arc_threshold").as_double();
    landmark_detector_->cluster_eps =
        this->get_parameter("cluster_eps").as_double();
    landmark_detector_->min_cluster_points =
        this->get_parameter("min_cluster_points").as_int();
    landmark_detector_->diameter_min =
        this->get_parameter("diameter_min").as_double();
    landmark_detector_->diameter_max =
        this->get_parameter("diameter_max").as_double();
    landmark_detector_->residual_real =
        this->get_parameter("residual_real").as_double();
    landmark_detector_->arc_min_points =
        this->get_parameter("arc_min_points").as_int();
  }

protected:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {

    landmark_localization::LaserScan scan;
    scan.header = scan_msg->header.frame_id;
    scan.ranges = scan_msg->ranges;
    scan.intensities = scan_msg->intensities;
    scan.angle_min = scan_msg->angle_min;
    scan.angle_max = scan_msg->angle_max;
    scan.angle_increment = scan_msg->angle_increment;
    scan.scan_time = scan_msg->scan_time;
    scan.range_min = scan_msg->range_min;
    scan.range_max = scan_msg->range_max;
    auto detected_posts = landmark_detector_->detect_circles(scan);

    if (detected_posts.empty()) {
      RCLCPP_INFO_STREAM(this->get_logger(), "未检测到有效反光柱");
      return;
    }

    // 发布landmark_lists 和 marker
    // 发布检测结果使用VisualMarker
    visualization_msgs::msg::MarkerArray marker_array;
    int landmarkId = 0;
    for (auto &landmark : detected_posts) {
      // 发布可视化标记
      geometry_msgs::msg::Pose landmark_pose =
          transforms::ToGeometryMsgPose(landmark.pose.pose);
      marker_array.markers.push_back(CreateLandmarkMarker(
          landmarkId++, landmark_pose, scan_msg->header.frame_id,
          scan_msg->header.stamp));
    }
    // 发布reflectors可视化标记
    marker_publisher_->publish(marker_array);
    return;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  std::shared_ptr<landmark_localization::ReflectivePostDetector>
      landmark_detector_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double landmark_rotation_weight_;
  double landmark_translation_weight_;

  std::mutex reflector_bars_mutex_;

  visualization_msgs::msg::Marker
  CreateLandmarkMarker(int landmark_index,
                       const geometry_msgs::msg::Pose &landmark_pose,
                       const std::string &frame_id, rclcpp::Time scan_time) {
    double kLandmarkMarkerScale = 0.1;
    visualization_msgs::msg::Marker marker;
    marker.ns = "reflective_circles";
    marker.id = landmark_index;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.stamp = scan_time;
    marker.header.frame_id = frame_id;
    marker.scale.x = kLandmarkMarkerScale;
    marker.scale.y = kLandmarkMarkerScale;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.6;
    marker.pose = landmark_pose;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReflectorDetectorCircle>();
  // auto match_assigner_ = std::make_shared<MatchAssigner>(node);
  // node->add_match_assigner(match_assigner_);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
