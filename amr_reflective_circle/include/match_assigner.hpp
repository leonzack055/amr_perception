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

#include "landmark_localization/landmark_assigner.hpp"
#include "landmark_localization/common/reflector_common.hpp"

class MatchAssigner {
 public:
  MatchAssigner(const rclcpp::Node::SharedPtr& node);

  // 新增：可视化功能
  void publishGlobalReflectorBars();
  visualization_msgs::msg::Marker createReflectorBarMarker(
      const ReflectorBar& reflector_bar, const std::string& frame_id);
  visualization_msgs::msg::Marker createTextMarker(
      const ReflectorBar& reflector_bar, const std::string& frame_id);

 
 private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      landmark_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_;

  // 新增：可视化发布器和定时器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      global_reflector_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

#endif  // MATCH_ASSIGNER_HPP_