#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "landmark_localization/landmark_reader.hpp"
#include "landmark_localization/landmark_matcher.hpp"
#include "landmark_localization/reflective_post_detector.hpp"

namespace landmark_localization {

class LandmarkLocalizationNode : public rclcpp::Node {
public:
  LandmarkLocalizationNode();
  ~LandmarkLocalizationNode();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void loadPriorLandmarks();
  void publishLandmarks(const std::vector<Landmark>& matched_landmarks,
                       const builtin_interfaces::msg::Time& stamp);
  void publishVisualizationMarkers(const std::vector<Landmark>& landmarks,
                                  const builtin_interfaces::msg::Time& stamp);
  
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr landmark_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // TF2 components
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Landmark processing components
  std::shared_ptr<LandmarkReader> landmark_reader_;
  std::shared_ptr<LandmarkMatcher> landmark_matcher_;
  std::shared_ptr<ReflectivePostDetector> post_detector_;
  
  // Parameters
  std::string pbstream_file_;
  std::string lidar_frame_;
  std::string map_frame_;
  double matching_threshold_;
  bool publish_visualization_;
  int intensity_threshold_use = 1000; // Whether to use simulation parameters for the detector
  double tf_time_tolerance_ = 0.05; 
  
  // Prior landmarks from map
  std::vector<LandmarkInfo> prior_landmarks_;
};

} // namespace landmark_localization