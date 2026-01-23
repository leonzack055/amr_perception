#include "amr_reflector_noise_handling/types/msg_conversion.hpp"

namespace transforms{

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3 & vector3)
{
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion & quaternion)
{
  return Eigen::Quaterniond(
    quaternion.w, quaternion.x, quaternion.y,
    quaternion.z);
}

Rigid3d ToRigid3d(const geometry_msgs::msg::TransformStamped & transform)
{
  return Rigid3d(
    ToEigen(transform.transform.translation),
    ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::msg::Pose & pose)
{
  return Rigid3d(
    {pose.position.x, pose.position.y, pose.position.z},
    ToEigen(pose.orientation));
}


geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d & vector3d)
{
  geometry_msgs::msg::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

geometry_msgs::msg::Transform ToGeometryMsgTransform(const Rigid3d & rigid3d)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

geometry_msgs::msg::Pose ToGeometryMsgPose(const Rigid3d & rigid3d)
{
  geometry_msgs::msg::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}
} // namespace transforms