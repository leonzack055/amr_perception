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

    publisher_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
      "/landmark", 10);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/reflector_markers", 10);

    RCLCPP_INFO(this->get_logger(), "反光柱检测节点初始化完成");

    // 获取参数
    landmark_rotation_weight_ = this->get_parameter("landmark_rotation_weight").as_double();
    landmark_translation_weight_ = this->get_parameter("landmark_translation_weight").as_double();

    // 1. 创建反光柱检测器
    landmark_detector_ = std::make_shared<landmark_localization::ReflectivePostDetector>();
    // 2. 创建反光柱匹配器
    landmark_assigner_ = std::make_shared<landmark_localization::LandmarkAssigner>();
    // 3. 创建tf缓存机制
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(
      this->get_clock(), tf2::durationFromSec(kTfBufferCacheTimeInSeconds));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

  void init()
  {
    // 1. 提取检测器参数
    // landmark_detector_->set_intensity_threshold_use(this->get_parameter("intensity_threshold_use").as_int());
    // landmark_detector_->set_cluster_eps(this->get_parameter("cluster_eps").as_double());
    // landmark_detector_->set_min_cluster_points(this->get_parameter("min_cluster_points").as_int());
    // landmark_detector_->set_diameter_min(this->get_parameter("diameter_min").as_double());
    // landmark_detector_->set_diameter_max(this->get_parameter("diameter_max").as_double());
    // landmark_detector_->set_residual_avg_threshold(this->get_parameter("residual_avg_threshold").as_double());
    // landmark_detector_->set_residual_std_threshold(this->get_parameter("residual_std_threshold").as_double());
    // landmark_detector_->set_residual_max_threshold(this->get_parameter("residual_max_threshold").as_double());
    // landmark_detector_->set_stat_mean_k(this->get_parameter("stat_mean_k").as_int());
    // landmark_detector_->set_stat_std_threshold(this->get_parameter("stat_std_threshold").as_double());
    // landmark_detector_->set_max_history_age(this->get_parameter("max_history_age").as_int());

    // 2. 提取landmark匹配器参数
    landmark_assigner_->search_range(10.0).match_threshold(0.2);

  }

protected:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    return;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  std::shared_ptr<landmark_localization::ReflectivePostDetector> landmark_detector_;
  std::shared_ptr<landmark_localization::LandmarkAssigner> landmark_assigner_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double landmark_rotation_weight_;
  double landmark_translation_weight_;
  int detection_id_counter_;
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
