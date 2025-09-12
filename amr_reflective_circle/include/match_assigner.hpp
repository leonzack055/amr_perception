#ifndef MATCH_ASSIGNER_HPP_
#define MATCH_ASSIGNER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cartographer_ros_msgs/msg/landmark_entry.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>

#include "reflector_common.hpp"

class MatchAssigner {
 public:
  MatchAssigner(const rclcpp::Node::SharedPtr& node);
  bool isLandmarkDetectorOK();
  void setLaserFrame(const std::string& laser_frame);
  geometry_msgs::msg::PoseStamped fusePoses(
      const geometry_msgs::msg::PoseStamped& pose1,
      const geometry_msgs::msg::PoseStamped& pose2, double weight1,
      double weight2);
  std::vector<Detection> getGlobalDetections(
      const std::vector<Detection>& current_detections);
  std::vector<Detection> updateTemporalFilter(
      const std::vector<Detection>& current_detections,
      const std::vector<int>& matched_bars);
  std::vector<ReflectorBar> assignLandmarkToReflectorBar(
      const std::vector<Detection>& current_detections);
  Eigen::Quaterniond CorrectOrientationToLaser();

  // 新增：可视化功能
  void publishGlobalReflectorBars();
  visualization_msgs::msg::Marker createReflectorBarMarker(
      const ReflectorBar& reflector_bar, const std::string& frame_id);
  visualization_msgs::msg::Marker createTextMarker(
      const ReflectorBar& reflector_bar, const std::string& frame_id);

 protected:
  bool time_check(rclcpp::Time time1, rclcpp::Time time2, double threshold);

 private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  std::vector<int> matchCurrentToHistory(
      const std::vector<Detection>& current_detections);
  void landmark_callback(
      const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      landmark_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_;
  ReflectorBarMap reflector_bars_;
  double search_range_;
  double match_threshold_;
  int id_counter_ = 0;
  bool base_to_laser_transform_available_ = false;
  transforms::Rigid3d base_to_laser_transform_;
  std::string laser_frame_;

  // 新增：可视化发布器和定时器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      global_reflector_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  // mutex
  std::mutex reflector_bars_mutex_;
};

#endif  // MATCH_ASSIGNER_HPP_