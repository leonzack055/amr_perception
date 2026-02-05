#include "amr_reflector_noise_handling/visualization_helper.hpp"
#include "amr_reflector_noise_handling/types/msg_conversion.hpp"

namespace amr_reflector_noise_handling {
VisualizationHelper::VisualizationHelper(const rclcpp::Node::SharedPtr &node)
    : node_(node) {
  node_->declare_parameter("enbale_laser_pub", true);
  node_->declare_parameter("use_global_frame", false);
  node_->declare_parameter("coordinate_frame", "laser");
  node_->declare_parameter("pub_laser_topic", "/scan");
  node_->declare_parameter("pub_filtered_topic", "/filtered_pointcould");
  node_->declare_parameter("pub_clustered_topic", "/clustered_pointcloud");
  node_->declare_parameter("pub_reflector_topic", "/detected_reflectors");
  node_->declare_parameter("pub_trakced_reflector_topic",
                           "/tracked_reflectors");
  node_->declare_parameter("pub_laser_trajectory_topic", "/laser_trajectory");
  node_->declare_parameter("pub_laser_pose_topic", "/laser_pose");
  // 加载消息
  LoadConfigFromNode();
  CreatePublisher();
}
VisualizationHelper::VisualizationHelper(const rclcpp::Node::SharedPtr &node,
                                         const Config &config)
    : node_(node), config_(config) {
  // 直接声明可视化消息
  CreatePublisher();
}
VisualizationHelper::~VisualizationHelper() {}

void VisualizationHelper::LoadConfigFromNode() {
  Config config;
  config.enbale_laser_pub = node_->get_parameter("enbale_laser_pub").as_bool();
  config.use_global_frame = node_->get_parameter("use_global_frame").as_bool();
  config.coordinate_frame =
      node_->get_parameter("coordinate_frame").as_string();
  config.pub_laser_topic = node_->get_parameter("pub_laser_topic").as_string();
  config.pub_filtered_topic =
      node_->get_parameter("pub_filtered_topic").as_string();
  config.pub_clustered_topic =
      node_->get_parameter("pub_clustered_topic").as_string();
  config.pub_reflector_topic =
      node_->get_parameter("pub_reflector_topic").as_string();
  config.pub_trakced_reflector_topic =
      node_->get_parameter("pub_trakced_reflector_topic").as_string();
  config.pub_laser_trajectory_topic =
      node_->get_parameter("pub_laser_trajectory_topic").as_string();
  config.pub_laser_pose_topic =
      node_->get_parameter("pub_laser_pose_topic").as_string();
  config_ = config;
  RCLCPP_INFO_STREAM(node_->get_logger(), "完成可视化配置参数加载:  ..... ");
  if (config_.use_global_frame) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[-]   可视化配置使用，全局坐标系: "
                                                << config.coordinate_frame);
    RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "[✘]   可视化使用可视化配置使用，全局坐标系, 不发布原始LaserScan: "
            << config.pub_laser_topic);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "[✔]   可视化使用全局坐标系, 发布LaserTrajectory: "
                           << config.pub_laser_trajectory_topic);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "[✔]   可视化使用全局坐标系, 发布LaserPose: "
                           << config.pub_laser_pose_topic);
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[✔]   可视化配置使用，激光坐标系: "
                                                << config.coordinate_frame);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "[✔]   可视化使用激光坐标系, 发布原始LaserScan: "
                           << config.pub_laser_topic);
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "[✘]   可视化使用激光坐标系, 不发布LaserTrajectory: "
                           << config.pub_laser_trajectory_topic);
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "[✘]   可视化使用激光坐标系, 不发布LaserPose: "
                           << config.pub_laser_pose_topic);
  }

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "[✔]   可视化使用全局坐标系, 发布LaserTrajectory: "
                         << config.pub_laser_trajectory_topic);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[✔]   可视化发布-*矫正后点云*-: "
                                              << config.pub_filtered_topic);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[✔]   可视化发布-*聚类后点云*-: "
                                              << config.pub_clustered_topic);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[✔]   可视化发布-*检测反光柱*-: "
                                              << config.pub_reflector_topic);
  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "[✔]   可视化发布-*跟踪反光柱*-: " << config.pub_trakced_reflector_topic);
}
void VisualizationHelper::CreatePublisher() {
  if (config_.use_global_frame) {
    // 不创建laser_pub
    laser_pub_ = nullptr;
    trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        config_.pub_laser_trajectory_topic, 10);
    laser_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        config_.pub_laser_pose_topic, 10);
    RCLCPP_WARN(node_->get_logger(),
                "使用全局坐标系, 不创建LaserScan消息发布器");
  } else {
    // 不创建轨迹发布器
    laser_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
        config_.pub_laser_topic, 10);
    trajectory_pub_ = nullptr;
    RCLCPP_WARN(node_->get_logger(), "使用激光坐标系, 不创建激光轨迹发布器");
    laser_pose_pub_ = nullptr;
  }
  // 完成扭曲补偿和强度过滤后的点云
  filtered_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.pub_filtered_topic, 10);
  cluster_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.pub_clustered_topic, 10);
  // 近似反光柱形状插值补偿
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.pub_reflector_topic, 10);
  tracked_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          config_.pub_trakced_reflector_topic, 10);
}

sensor_msgs::msg::PointCloud2 VisualizationHelper::CopyPointsToPointCloud2(
    const std::vector<Point> &points, const transforms::Rigid3d &global_pose) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = node_->now();
  cloud_msg.header.frame_id = config_.coordinate_frame;
  cloud_msg.height = 1;
  cloud_msg.width = points.size();

  cloud_msg.fields.resize(4);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;

  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count = 1;

  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[2].count = 1;

  cloud_msg.fields[3].name = "intensity";
  cloud_msg.fields[3].offset = 12;
  cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[3].count = 1;

  cloud_msg.is_bigendian = false;
  cloud_msg.point_step = 16;
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step);

  for (size_t i = 0; i < points.size(); ++i) {
    uint8_t *ptr = &cloud_msg.data[i * cloud_msg.point_step];
    Eigen::Vector3d point = Eigen::Vector3d(points[i].x, points[i].y, 0.0);

    if (config_.use_global_frame) {
      point = global_pose * Eigen::Vector3d(points[i].x, points[i].y, 0.0);
    }

    float x = static_cast<float>(point.x());
    std::memcpy(ptr + 0, &x, sizeof(float));

    float y = static_cast<float>(point.y());
    std::memcpy(ptr + 4, &y, sizeof(float));

    float z = 0.0f;
    std::memcpy(ptr + 8, &z, sizeof(float));

    float intensity = static_cast<float>(points[i].intensity);
    std::memcpy(ptr + 12, &intensity, sizeof(float));
  }
  return cloud_msg;
}

void VisualizationHelper::pulishOriginLaserScan(
    const sensor_msgs::msg::LaserScan::Ptr &laser_msg) {
  if (!laser_pub_)
    return;
  laser_msg->header.stamp = node_->now();
  laser_pub_->publish(*laser_msg);
}
void VisualizationHelper::publishFilteredPointCloud(
    const std::vector<Point> &points, const transforms::Rigid3d &global_pose) {
  sensor_msgs::msg::PointCloud2 cloud_msg =
      CopyPointsToPointCloud2(points, global_pose);
  filtered_pub_->publish(cloud_msg);
}
void VisualizationHelper::publishClusteredPointCloud(
    const std::vector<Point> &points, const transforms::Rigid3d &global_pose) {
  sensor_msgs::msg::PointCloud2 cloud_msg =
      CopyPointsToPointCloud2(points, global_pose);
  cluster_points_pub_->publish(cloud_msg);
}

void VisualizationHelper::publishReflectorMarkers(
    const std::vector<DetectedReflector> &reflectors,
    const transforms::Rigid3d &global_pose) {
  visualization_msgs::msg::MarkerArray marker_array;
  // Resize to accommodate both cylinder markers and text labels
  marker_array.markers.resize(reflectors.size() * 3);

  for (size_t i = 0; i < reflectors.size(); ++i) {
    // Create cylinder marker for the reflector
    visualization_msgs::msg::Marker cylinder_marker;
    cylinder_marker.header.stamp = node_->now();
    cylinder_marker.header.frame_id = config_.coordinate_frame;
    cylinder_marker.ns = "reflective_posts";
    cylinder_marker.id = i * 3; // Even IDs for cylinders
    cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder_marker.action = visualization_msgs::msg::Marker::ADD;

    cylinder_marker.pose.position.x = reflectors[i].center.x;
    cylinder_marker.pose.position.y = reflectors[i].center.y;
    if (config_.use_global_frame) {
      Eigen::Vector3d global_reflector_pos =
          global_pose *
          Eigen::Vector3d(reflectors[i].center.x, reflectors[i].center.y, 0.0);
      cylinder_marker.pose.position.x = global_reflector_pos.x();
      cylinder_marker.pose.position.y = global_reflector_pos.y();
    }
    cylinder_marker.pose.position.z = 0.0;
    cylinder_marker.pose.orientation.w = 1.0;

    cylinder_marker.scale.x = 0.001;
    cylinder_marker.scale.y = 0.001;
    cylinder_marker.scale.z = 0.5;

    double confidence = reflectors[i].confidence;
    cylinder_marker.color.r = 0.0;
    cylinder_marker.color.g = confidence;
    cylinder_marker.color.b = 1.0 - confidence;
    cylinder_marker.color.a = 0.8;

    cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker_array.markers[i * 3] = cylinder_marker;

    // 创建检测出来的拟合圆
    visualization_msgs::msg::Marker cylinder_marker2;
    cylinder_marker2.header.stamp = node_->now();
    cylinder_marker2.header.frame_id = config_.coordinate_frame;
    cylinder_marker2.ns = "reflective_posts";
    cylinder_marker2.id = i * 3 + 1; // Even IDs for cylinders
    cylinder_marker2.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder_marker2.action = visualization_msgs::msg::Marker::ADD;

    cylinder_marker2.pose.position.x = cylinder_marker.pose.position.x;
    cylinder_marker2.pose.position.y = cylinder_marker.pose.position.y;
    cylinder_marker2.pose.position.z = 0.0;
    cylinder_marker2.pose.orientation.w = 1.0;

    cylinder_marker2.scale.x = reflectors[i].diameter;
    cylinder_marker2.scale.y = reflectors[i].diameter;
    cylinder_marker2.scale.z = 0.5;

    cylinder_marker2.color.r = 1.0;
    cylinder_marker2.color.g = 0.0;
    cylinder_marker2.color.b = 1.0;
    cylinder_marker2.color.a = 0.8;

    cylinder_marker2.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers[i * 3 + 1] = cylinder_marker2;

    // Create text label marker for the reflector idx
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.stamp = node_->now();
    text_marker.header.frame_id = config_.coordinate_frame;
    text_marker.ns = "reflective_posts";
    text_marker.id = i * 3 + 2; // Odd IDs for text labels
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = cylinder_marker.pose.position.x;
    text_marker.pose.position.y = cylinder_marker.pose.position.y;
    text_marker.pose.position.z = 0.6; // Position text above the cylinder
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.z = 0.3; // Text height

    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    // Set text to display the reflector idx
    text_marker.text = "R" + std::to_string(reflectors[i].idx);

    text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker_array.markers[i * 3 + 2] = text_marker;
  }
  marker_pub_->publish(marker_array);
}
void VisualizationHelper::publishTrackedReflectorMarkers(
    const std::vector<TrackedReflector> &all_reflectors,
    const transforms::Rigid3d &global_pose) {
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(all_reflectors.size() * 3);

  for (size_t i = 0; i < all_reflectors.size(); ++i) {
    const auto &tracker = all_reflectors[i];
    // Color based on state
    std_msgs::msg::ColorRGBA color;
    if (tracker.state == TrackedReflector::CONFIRMED) {
      // Green for confirmed reflectors
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 0.8;
    } else if (tracker.state == TrackedReflector::TENTATIVE) {
      // Yellow for tentative reflectors
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 0.6;
    } else if (tracker.state == TrackedReflector::INACTIVE) {
      // Blue for inactive reflectors
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.8;
    }
    // reflector位置
    Eigen::Vector3d reflector_pos = Eigen::Vector3d(
        tracker.filtered_position.x, tracker.filtered_position.y, 0.0);
    Eigen::Vector3d global_relector_pos = reflector_pos;
    if (!config_.use_global_frame) {
      global_relector_pos = global_pose.inverse() * reflector_pos;
    }

    // Create sphere marker for the tracked position
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.stamp = node_->now();
    sphere_marker.header.frame_id = config_.coordinate_frame;
    sphere_marker.ns = "tracked_reflectors";
    sphere_marker.id = i * 3;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;

    sphere_marker.pose.position.x = global_relector_pos.x();
    sphere_marker.pose.position.y = global_relector_pos.y();
    sphere_marker.pose.position.z = 0.0;
    sphere_marker.pose.orientation.w = 1.0;

    sphere_marker.scale.x = tracker.diameter;
    sphere_marker.scale.y = tracker.diameter;
    sphere_marker.scale.z = 0.5;

    sphere_marker.color = color;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers[i * 3] = sphere_marker;

    // Create text label for global ID
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.stamp = node_->now();
    text_marker.header.frame_id = config_.coordinate_frame;
    text_marker.ns = "tracked_reflectors";
    text_marker.id = i * 3 + 1;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = global_relector_pos.x();
    text_marker.pose.position.y = global_relector_pos.y();
    text_marker.pose.position.z = 0.6;
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.z = 0.3;

    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    // Display global ID and state
    std::string state_str;
    if (tracker.state == TrackedReflector::CONFIRMED) {
      state_str = "C";
    } else if (tracker.state == TrackedReflector::TENTATIVE) {
      state_str = "T";
    } else {
      state_str = "I";
    }
    text_marker.text = "G" + std::to_string(tracker.global_id) + "[" +
                       state_str + "] " +
                       std::to_string(tracker.total_detection_count) + "d";

    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers[i * 3 + 1] = text_marker;

    // Create uncertainty circle (position standard deviation)
    visualization_msgs::msg::Marker uncertainty_marker;
    uncertainty_marker.header.stamp = node_->now();
    uncertainty_marker.header.frame_id = config_.coordinate_frame;
    uncertainty_marker.ns = "tracked_reflectors";
    uncertainty_marker.id = i * 3 + 2;
    uncertainty_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    uncertainty_marker.action = visualization_msgs::msg::Marker::ADD;

    uncertainty_marker.pose.position.x = global_relector_pos.x();
    uncertainty_marker.pose.position.y = global_relector_pos.y();
    uncertainty_marker.pose.position.z = 0.0;
    uncertainty_marker.pose.orientation.w = 1.0;

    // Show uncertainty as 2x standard deviation
    double uncertainty_radius = 2.0 * tracker.position_std_dev;
    uncertainty_marker.scale.x = uncertainty_radius * 2.0;
    uncertainty_marker.scale.y = uncertainty_radius * 2.0;
    uncertainty_marker.scale.z = 0.02;

    uncertainty_marker.color.r = 0.5;
    uncertainty_marker.color.g = 0.5;
    uncertainty_marker.color.b = 0.0;
    uncertainty_marker.color.a = 0.3;

    uncertainty_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers[i * 3 + 2] = uncertainty_marker;
  }

  tracked_marker_pub_->publish(marker_array);
}

void VisualizationHelper::publishLaserPose(
    const transforms::Rigid3d &global_pose) {
  if (!laser_pose_pub_)
    return;
  // Publish the laser pose
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = node_->now();
  pose_msg.header.frame_id = config_.coordinate_frame;
  pose_msg.pose = transforms::ToGeometryMsgPose(global_pose);
  laser_pose_pub_->publish(pose_msg);
}

} // namespace amr_reflector_noise_handling