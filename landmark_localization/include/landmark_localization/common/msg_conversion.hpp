/**
 * @file msg_conversion.hpp
 * @brief 消息转换函数, 将Rigid3d 与 Ros相关的msg进行互转
 * @author LeonZack
 * @date 2025-09-21
 */

#ifndef LANDMARK_LOCALIZATION_COMMON_MSG_CONVERSION_HPP_
#define LANDMARK_LOCALIZATION_COMMON_MSG_CONVERSION_HPP_

#include "reflector_common.hpp"
#include <Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tuple>
#include <functional>
#include <fstream>
#include <chrono>

namespace transforms{
Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3 & vector3);
Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion & quaternion);

Rigid3d ToRigid3d(const geometry_msgs::msg::TransformStamped & transform);
Rigid3d ToRigid3d(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d & vector3d);
geometry_msgs::msg::Transform ToGeometryMsgTransform(const Rigid3d & rigid3d);
geometry_msgs::msg::Pose ToGeometryMsgPose(const Rigid3d & rigid3d);
}

#endif  // LANDMARK_LOCALIZATION_COMMON_MSG_CONVERSION_HPP_