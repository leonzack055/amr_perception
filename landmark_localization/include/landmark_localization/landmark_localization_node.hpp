#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "landmark_localization/landmark_reader.hpp"
#include "landmark_localization/landmark_matcher.hpp"
#include "landmark_localization/reflective_post_detector.hpp"
#include "amr_ros_msg/msg/pose_with_type_stamped.hpp"
#include <yaml-cpp/yaml.h>

namespace landmark_localization {

class LandmarkLocalizationNode : public rclcpp::Node {
public:
  LandmarkLocalizationNode();
  ~LandmarkLocalizationNode();

private:
  void loadParametersFromYaml(const std::string& yaml_file_path);
  void laserScan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void laserScan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void initialPoseCallback(const amr_ros_msg::msg::PoseWithTypeStamped::SharedPtr msg);
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
  rclcpp::Subscription<amr_ros_msg::msg::PoseWithTypeStamped>::SharedPtr initial_pose_sub_;
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
  std::string landmark_topic_;
  std::string visualization_topic_;
  std::string landmark_localization_topic_;
  std::string initial_pose_topic_;
  bool use_combine_ = false; // 是否启用双雷达联合处理
  
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
  int intensity_threshold_use = 1000;
  int max_age_param = 8;
  double tf_time_tolerance_ = 0.05;
  int min_landmarks_for_pose_ = 3; 
  
  // Prior landmarks from map
  std::vector<LandmarkInfo> prior_landmarks_;

  // 反光柱ID组合滤波
  bool use_calculate_filter_ = false;
  int filter_num_ = 5;
  std::vector<std::set<std::string>> recent_landmark_sets_;
  int consecutive_count_ = 0;
  std::set<std::string> last_accepted_set_; 
  bool checkFilterCondition(const std::vector<LandmarkInfo>& selected_prior_landmarks);
};

} // namespace landmark_localization