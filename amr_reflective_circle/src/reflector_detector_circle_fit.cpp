#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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

#include "landmark_localization/reflective_post_detector.hpp"
#include "landmark_localization/landmark_assigner.hpp"
#include "landmark_localization/common/msg_conversion.hpp"
#include "match_assigner.hpp"

using namespace std::chrono_literals;

//std::ofstream outFile("./test_cpp_circle_r.txt");

class ReflectorDetectorCircle : public rclcpp::Node
{
public:
  ReflectorDetectorCircle()
  : Node("reflector_detector_maker")
  {
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
      "/scan", 10, std::bind(&ReflectorDetectorCircle::scan_callback, this, std::placeholders::_1));

    landmark_list_publisher_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
      "/landmark", 10);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/reflector_markers", 10);

    tracked_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tracked_pose", 10,
      std::bind(&ReflectorDetectorCircle::tracked_pose_callback, this, std::placeholders::_1));

    landmark_sub_ =
      this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/landmark_poses_list", 10,
      std::bind(
        &ReflectorDetectorCircle::optimized_landmark_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "反光柱检测节点初始化完成");

    // 1. 创建反光柱检测器
    landmark_detector_ = std::make_shared<landmark_localization::ReflectivePostDetector>(
      this->get_parameter(
        "intensity_threshold_use").as_int());
    // 2. 创建反光柱匹配器
    landmark_assigner_ = std::make_shared<landmark_localization::LandmarkAssigner>();

    // 3. 创建tf缓存机制
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(
      this->get_clock(), tf2::durationFromSec(kTfBufferCacheTimeInSeconds));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 3. 创建wall_timer发布assigner保存的全部landmark
    // visualization_timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(500),  // 每500ms发布一次
    //   [this]() {
    //     publishGlobalReflectorBars();
    //   });
    init();
  }

  void init()
  {
    // 获取参数
    landmark_rotation_weight_ = this->get_parameter("landmark_rotation_weight").as_double();
    landmark_translation_weight_ = this->get_parameter("landmark_translation_weight").as_double();

    // 1. 提取检测器参数
    landmark_detector_->intensity_threshold_use =
      this->get_parameter("intensity_threshold_use").as_int();
    landmark_detector_->arc_threshold = this->get_parameter("arc_threshold").as_double();
    landmark_detector_->cluster_eps = this->get_parameter("cluster_eps").as_double();
    landmark_detector_->min_cluster_points = this->get_parameter("min_cluster_points").as_int();
    landmark_detector_->diameter_min = this->get_parameter("diameter_min").as_double();
    landmark_detector_->diameter_max = this->get_parameter("diameter_max").as_double();
    landmark_detector_->residual_real = this->get_parameter("residual_real").as_double();
    landmark_detector_->arc_min_points = this->get_parameter("arc_min_points").as_int();


    // landmark_detector_->residual_avg_threshold = this->get_parameter("residual_avg_threshold").as_double();
    // landmark_detector_->residual_std_threshold = this->get_parameter("residual_std_threshold").as_double();
    // landmark_detector_->residual_max_threshold = this->get_parameter("residual_max_threshold").as_double();
    // landmark_detector_->stat_mean_k = this->get_parameter("stat_mean_k").as_int();
    // landmark_detector_->stat_std_threshold = this->get_parameter("stat_std_threshold").as_double();
    // landmark_detector_->max_history_age = this->get_parameter("max_history_age").as_int();

    // landmark_detector_->match_distance_threshold = this->get_parameter("match_distance_threshold").as_double();
    // landmark_detector_->max_arc_feature = this->get_parameter("max_arc_feature").as_double();

    // 2. 提取landmark匹配器参数
    landmark_assigner_->search_range(10.0).match_threshold(0.2);

  }

protected:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (!landmark_assigner_->isBase2LaserTransOK()) {
      geometry_msgs::msg::TransformStamped transform;
      try {
        tf_buffer_->canTransform("base_link", scan_msg->header.frame_id, tf2::TimePointZero);
        transform = tf_buffer_->lookupTransform(
          "base_link",
          scan_msg->header.frame_id,
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(), "TF2 查找 base_link 2 %s error: %s",
          scan_msg->header.frame_id.c_str(), ex.what());
      }
      landmark_assigner_->setBase2LaserTrans(transforms::ToRigid3d(transform));
      RCLCPP_WARN_STREAM(
        this->get_logger(), "反光柱检测器分配器设置base_link To " << scan_msg->header.frame_id << " 变换");
      return;
    }

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

    if (!landmark_assigner_->isLandmarkDetectorOK(scan_msg->header.stamp.nanosec)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "反光柱分配器未初始化完成!! 请检查TF和tracked_pose!");
      return;
    }

    if (detected_posts.empty()) {
      RCLCPP_INFO_STREAM(this->get_logger(), "未检测到有效反光柱");
      return;
    }

    auto reflector_posts = landmark_assigner_->assignLandmarkToReflectorBar(detected_posts);

    // 发布landmark_lists 和 marker
    cartographer_ros_msgs::msg::LandmarkList landmark_list_msg;
    std::vector<cartographer_ros_msgs::msg::LandmarkEntry> poses_array;
    cartographer_ros_msgs::msg::LandmarkEntry poseSimple;
    landmark_list_msg.header = scan_msg->header;
    // 发布检测结果使用VisualMarker
    visualization_msgs::msg::MarkerArray marker_array;
    for (auto & landmark : reflector_posts) {
      poseSimple.tracking_from_landmark_transform = transforms::ToGeometryMsgPose(
        landmark.g_detection_.pose.pose);
      poseSimple.translation_weight = landmark.g_detection_.translationW *
        landmark_translation_weight_;
      poseSimple.rotation_weight = landmark_rotation_weight_;
      poseSimple.id = landmark.id_str_;
      poses_array.push_back(poseSimple);
      // 发布可视化标记
      marker_array.markers.push_back(
        CreateLandmarkMarker(
          landmark.id_, poseSimple.tracking_from_landmark_transform,
          scan_msg->header.frame_id, scan_msg->header.stamp));
    }
    landmark_list_msg.landmarks = poses_array;
    landmark_list_publisher_->publish(landmark_list_msg);
    // 发布reflectors可视化标记
    marker_publisher_->publish(marker_array);
    return;
  }


  void tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    landmark_assigner_->update_tracked_pose(
      transforms::ToRigid3d(
        pose_msg->pose), pose_msg->header.stamp.nanosec);
  }

  void optimized_landmark_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::map<int, transforms::Rigid3d> optimized_landmarks;
    int64_t optimized_timestamp;
    for (const auto & marker_msg : msg->markers) {
      // 处理反光柱位姿
      if (marker_msg.ns == "Landmarks" && marker_msg.header.frame_id == "map") {
        int id = marker_msg.id;
        optimized_landmarks[id] = transforms::ToRigid3d(marker_msg.pose);
      }
    }
    if (optimized_landmarks.empty()) {
      return;
    }
    optimized_timestamp = msg->markers[0].header.stamp.nanosec;
    RCLCPP_INFO_STREAM(this->get_logger(), "反光柱分配器更新优化后的反光柱位姿");
    landmark_assigner_->update_landmarks(optimized_landmarks, optimized_timestamp);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_sub_; // 优化后柱子

  rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr landmark_list_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    global_reflector_publisher_;

  std::shared_ptr<landmark_localization::ReflectivePostDetector> landmark_detector_;
  std::shared_ptr<landmark_localization::LandmarkAssigner> landmark_assigner_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double landmark_rotation_weight_;
  double landmark_translation_weight_;

  std::mutex reflector_bars_mutex_;

  visualization_msgs::msg::Marker CreateLandmarkMarker(
    int landmark_index,
    const geometry_msgs::msg::Pose & landmark_pose,
    const std::string & frame_id,
    rclcpp::Time scan_time)
  {
    double kLandmarkMarkerScale = 0.05;
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

  void publishGlobalReflectorBars()
  {
    auto reflector_bars = this->landmark_assigner_->getReflectorBars();
    if (reflector_bars.empty()) {
      return; // 没有反光柱数据时不发布
    }
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time current_time = this->now();
    for (const auto & [id, reflector_bar] : reflector_bars) {
      visualization_msgs::msg::Marker marker;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.ns = "global_reflector_bars";
      marker.header.frame_id = "map";
      marker.id = reflector_bar.id_;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.header.stamp = current_time;
      marker.pose = transforms::ToGeometryMsgPose(reflector_bar.g_detection_.pose.pose);
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      if (reflector_bar.optimized_) {
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      } else {
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
      }
      marker_array.markers.push_back(marker);

      // 添加文本标记显示ID
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.stamp = current_time;
      text_marker.header.frame_id = "map";
      text_marker.ns = "assigner_bars";
      text_marker.id = reflector_bar.id_ + 10000; // 避免与实际landmark id冲突
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      // 设置位置（在反光柱上方）
      text_marker.pose = marker.pose;
      text_marker.pose.position.z = text_marker.pose.position.z + 0.3;// 在反光柱上方0.3米
      // 设置文本内容
      text_marker.text = "id:" + reflector_bar.id_str_;
      // 设置文本大小
      text_marker.scale.z = 0.2; // 文本高度
      // 设置颜色（白色）
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      // 如果是优化后的id，设为黄色
      if (reflector_bar.optimized_) {
        text_marker.color.r = 1.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 1.0;
      }

      // 设置生命周期
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
      marker_array.markers.push_back(text_marker);
    }
    // 发布可视化标记
    global_reflector_publisher_->publish(marker_array);
    return;
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReflectorDetectorCircle>();
  // auto match_assigner_ = std::make_shared<MatchAssigner>(node);
  // node->add_match_assigner(match_assigner_);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
