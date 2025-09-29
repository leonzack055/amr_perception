#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include "landmark_localization/landmark_reader.hpp"
#include "landmark_localization/landmark_matcher.hpp"
#include "landmark_localization/reflective_post_detector.hpp"

namespace landmark_localization {

class LandmarkLocalizationNode : public rclcpp::Node {
public:
  LandmarkLocalizationNode();
  ~LandmarkLocalizationNode();

private:
  void laserScan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void laserScan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void processCombinedScan(const sensor_msgs::msg::LaserScan::SharedPtr scan1_msg);
  bool transformScan2LandmarksToScan1Frame(const sensor_msgs::msg::LaserScan::SharedPtr scan2_msg,
                                          const builtin_interfaces::msg::Time& target_time,
                                          const std::string& target_frame,
                                          std::vector<Landmark>& transformed_landmarks);
  void loadPriorLandmarks();
  void publishLandmarks(const std::vector<Landmark>& matched_landmarks,
                       const builtin_interfaces::msg::Time& stamp,
                       const std::string& lidar_frame);
  void publishVisualizationMarkers(const std::vector<Landmark>& landmarks,
                                  const builtin_interfaces::msg::Time& stamp,
                                  const std::string& lidar_frame);
  bool calculateRobotPose(const std::vector<LandmarkInfo>& prior_landmarks,
                         const std::vector<Landmark>& detected_landmarks,
                         geometry_msgs::msg::PoseStamped& robot_pose,
                         const std::string& lidar_frame);
  
  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
  rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr landmark_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  
  // TF2 components
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Landmark processing components
  std::shared_ptr<LandmarkReader> landmark_reader_;
  std::shared_ptr<LandmarkMatcher> landmark_matcher_;
  std::shared_ptr<ReflectivePostDetector> post_detector_;
  
  // Parameters
  std::string pbstream_file_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  bool use_combined_processing_ = true; // 是否启用双雷达联合处理
  tf2::Transform lidar2_to_lidar1_tf_;
  bool has_lidar2_to_lidar1_tf_ = false;
  
  // 存储最新的scan2消息
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan2_msg_ = nullptr;
  std::mutex scan2_mutex_;
  
  // 激光雷达1参数
  std::string lidar1_frame_ = "";
  std::string scan1_topic_;
  bool use_lidar1_;
  tf2::Transform base_to_lidar1_tf_;
  bool has_base_to_lidar1_tf_ = false;
  
  // 激光雷达2参数
  std::string lidar2_frame_ = "";
  std::string scan2_topic_;
  bool use_lidar2_;
  tf2::Transform base_to_lidar2_tf_;
  bool has_base_to_lidar2_tf_ = false;
  
  double matching_threshold_;
  bool publish_visualization_;
  int intensity_threshold_use = 1600;
  double tf_time_tolerance_ = 0.05;
  int min_landmarks_for_pose_ = 3; 
  
  // Prior landmarks from map
  std::vector<LandmarkInfo> prior_landmarks_;
};

} // namespace landmark_localization