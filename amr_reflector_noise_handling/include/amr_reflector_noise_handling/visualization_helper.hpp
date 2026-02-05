#ifndef AMR_REFLECTOR_NOISE_HANDLING_VISUALIZATION_HELPER_HPP
#define AMR_REFLECTOR_NOISE_HANDLING_VISUALIZATION_HELPER_HPP
#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

namespace amr_reflector_noise_handling {

class VisualizationHelper {
public:
  struct Config {
    bool enbale_laser_pub = true;
    bool use_global_frame = false;
    std::string coordinate_frame = "laser";
    std::string pub_laser_topic = "/scan";
    std::string pub_filtered_topic = "/filtered_pointcould";
    std::string pub_clustered_topic = "/clustered_pointcloud";
    std::string pub_reflector_topic = "/reflectors";
    std::string pub_trakced_reflector_topic = "/tracked_reflectors";
    std::string pub_laser_trajectory_topic = "/laser_trajectory";
    std::string pub_laser_pose_topic = "/laser_pose";
  };
  VisualizationHelper(const rclcpp::Node::SharedPtr &node);
  VisualizationHelper(const rclcpp::Node::SharedPtr &node,
                      const Config &config);
  ~VisualizationHelper();
  void pulishOriginLaserScan(const sensor_msgs::msg::LaserScan::Ptr &laser_msg);
  void publishFilteredPointCloud(
      const std::vector<Point> &points,
      const transforms::Rigid3d &global_pose = transforms::Rigid3d::Identity());
  void publishClusteredPointCloud(
      const std::vector<Point> &points,
      const transforms::Rigid3d &global_pose = transforms::Rigid3d::Identity());
  /**
   * @brief Publish tracked reflector markers
   * 可选择以laser为基准，还是以全局map为基准
   * 1. Sphere:对于时间窗内确认的反光柱，显示为绿色；
   * 对于未确认的反光柱，设定为黄色； 已经确认但不在跟踪窗内，显示为绿色。
   * 2. Text: [C]跟踪到的确认反光柱； [T]: 检测到但未确认为反光柱；
   * [I]已经确认但不在跟踪窗范围内的反光柱
   * 3. Cylinder:
   * 显示为当前跟踪器内的放光柱的平滑不确定性信息；以反光柱位置的距离方法，构建平面圆。
   */
  void publishTrackedReflectorMarkers(
      const std::vector<TrackedReflector> &reflectors,
      const transforms::Rigid3d &global_pose = transforms::Rigid3d::Identity());
  /**
   * @brief Publish reflector markers with current timestamp
   */
  void publishReflectorMarkers(
      const std::vector<DetectedReflector> &all_reflectors,
      const transforms::Rigid3d &global_pose = transforms::Rigid3d::Identity());
  /**
   * 发布指定坐标系下的激光全局位姿
   */
  void publishLaserPose(
      const transforms::Rigid3d &global_pose = transforms::Rigid3d::Identity());

private:
  void LoadConfigFromNode();
  void CreatePublisher();
  sensor_msgs::msg::PointCloud2 CopyPointsToPointCloud2(
      const std::vector<Point> &points,
      const transforms::Rigid3d &global_pose = transforms::Rigid3d::Identity());
  Config config_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_pub_; // 发布激光雷达，支持Laser
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      filtered_pub_; // 支持Laser和全局坐标;
                     // 完成扭曲补偿和强度过滤后的点云
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cluster_points_pub_; // 聚类后的有效点云，用于生成拟合圆，支持Laser和全局坐标
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_; // 发布检测到的反光柱，支持Laser和全局坐标
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      tracked_marker_pub_; // 发布跟踪到的反光柱，支持Laser和全局坐标
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr laser_pose_pub_;
};

} // namespace amr_reflector_noise_handling

#endif // !AMR_REFLECTOR_NOISE_HANDLING_VISUALIZATION_HELPER_HPP
