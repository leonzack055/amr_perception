#include "landmark_localization/landmark_localization_node.hpp"
#include "landmark_localization/landmark_reader.hpp"
#include "landmark_localization/landmark_matcher.hpp"
#include "landmark_localization/reflective_post_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <Eigen/Dense> 
#include <Eigen/Geometry>

namespace landmark_localization {

using namespace std::chrono_literals;

LandmarkLocalizationNode::LandmarkLocalizationNode()
: Node("landmark_localization_node")
{
  // Declare parameters
  this->declare_parameter("pbstream_file", "");
  this->declare_parameter("lidar_frame", "two_d_lidar");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("matching_threshold", 0.5);
  this->declare_parameter("publish_visualization", true);
  this->declare_parameter("intensity_threshold_use", 1000);
  this->declare_parameter("tf_time_tolerance", 0.05);
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("min_landmarks_for_pose", 3);
  this->declare_parameter("scan_topic", "/scan");
  this->declare_parameter("landmark_topic", "/landmark");
  this->declare_parameter("visualization_topic", "/landmark_localization_markers");
  this->declare_parameter("landmark_localization_topic", "/landmark_tracked_pose");
  
  // Get parameters
  pbstream_file_ = this->get_parameter("pbstream_file").as_string();
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  matching_threshold_ = this->get_parameter("matching_threshold").as_double();
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  intensity_threshold_use = this->get_parameter("intensity_threshold_use").as_int();
  tf_time_tolerance_ = this->get_parameter("tf_time_tolerance").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();
  min_landmarks_for_pose_ = this->get_parameter("min_landmarks_for_pose").as_int();
  std::string scan_topic = this->get_parameter("scan_topic").as_string();
  std::string landmark_topic = this->get_parameter("landmark_topic").as_string();
  std::string visualization_topic = this->get_parameter("visualization_topic").as_string();
  std::string landmark_localization_topic = this->get_parameter("landmark_localization_topic").as_string();

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 在初始化时获取静态TF变换
  try {
    geometry_msgs::msg::TransformStamped base_to_lidar_tf_msg = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, tf2::TimePointZero);
    
    tf2::fromMsg(base_to_lidar_tf_msg.transform, base_to_lidar_tf_);
    has_base_to_lidar_tf_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Successfully loaded static TF from %s to %s", 
               base_frame_.c_str(), lidar_frame_.c_str());
    
    // 输出TF变换信息用于调试
    double roll, pitch, yaw;
    tf2::Matrix3x3(base_to_lidar_tf_.getRotation()).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "TF translation: x=%.3f, y=%.3f, z=%.3f",
               base_to_lidar_tf_.getOrigin().x(),
               base_to_lidar_tf_.getOrigin().y(),
               base_to_lidar_tf_.getOrigin().z());
    RCLCPP_INFO(this->get_logger(), "TF rotation: roll=%.3f, pitch=%.3f, yaw=%.3f",
               roll, pitch, yaw);
               
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
               base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    RCLCPP_WARN(this->get_logger(), "Will try to get TF transform in each callback");
  }
  
  // Initialize components
  landmark_reader_ = std::make_shared<LandmarkReader>(pbstream_file_);
  landmark_matcher_ = std::make_shared<LandmarkMatcher>(matching_threshold_);
  post_detector_ = std::make_shared<ReflectivePostDetector>(intensity_threshold_use);

  // Load prior landmarks from map
  loadPriorLandmarks();
  
  // Create subscribers and publishers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 10,
    std::bind(&LandmarkLocalizationNode::laserScanCallback, this, std::placeholders::_1));
  
  landmark_pub_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
    landmark_topic, 10);
  
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    landmark_localization_topic, 10);
  
  if (publish_visualization_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic, 10);
  }
  
  RCLCPP_INFO(this->get_logger(), "Landmark localization node started");
  RCLCPP_INFO(this->get_logger(), "Lidar frame: %s", lidar_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Matching threshold: %.2f m", matching_threshold_);
}

LandmarkLocalizationNode::~LandmarkLocalizationNode()
{
}

void LandmarkLocalizationNode::loadPriorLandmarks() {
  if (pbstream_file_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No pbstream file specified, using empty prior map");
    return;
  }
  
  if (landmark_reader_->readLandmarks()) {
    landmark_reader_->printLandmarks();
    
    prior_landmarks_ = landmark_reader_->getLandmarks();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu prior landmarks from map", prior_landmarks_.size());
    
    // Add landmarks to matcher
    for (const auto& landmark : prior_landmarks_) {
      // Convert LandmarkInfo to Landmark structure
      Landmark lm;
      try {
        lm.id = std::stoi(landmark.landmark_id);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to convert landmark ID '%s' to integer", 
                   landmark.landmark_id.c_str());
        continue;
      }
      lm.x = landmark.x;
      lm.y = landmark.y;
      landmark_matcher_->addPriorLandmark(lm);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load landmarks from pbstream file: %s", 
                pbstream_file_.c_str());
  }
}

void LandmarkLocalizationNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "---------------scan---------------");
  try {    
    // Get transform from lidar to map frame
    geometry_msgs::msg::TransformStamped transform;

    // 时间戳精确匹配，查找在完全相同的时间点下，从 lidar_frame_ 到 map_frame_ 的变换关系
    // transform = tf_buffer_->lookupTransform(map_frame_, lidar_frame_, msg->header.stamp);

    // 尝试获取最新可用的变换
    // transform = tf_buffer_->lookupTransform(map_frame_, lidar_frame_, tf2::TimePointZero);

    // 允许一定时间范围内的最近变换
    transform = tf_buffer_->lookupTransform(
        map_frame_, 
        lidar_frame_, 
        msg->header.stamp,
        tf2::durationFromSec(tf_time_tolerance_) // 时间容忍度, unit: s
    );

    // Convert ROS LaserScan to standard library LaserScan
    LaserScan scan;
    scan.ranges = msg->ranges;
    scan.intensities = msg->intensities;
    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    scan.scan_time = msg->scan_time;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    
    // Detect reflective posts in laser scan
    auto detected_posts = post_detector_->detect(scan);
    
    if (detected_posts.empty()) {
      std::cout << "未检测到有效反光柱" << std::endl;
      return;
    } else {
      std::cout << "检测到 " << detected_posts.size() << " 个反光柱" << std::endl;
    }

    // Convert detected posts to Landmark format for matching
    std::vector<Landmark> detected_landmarks;
    for (size_t i = 0; i < detected_posts.size(); ++i) {
      Landmark lm;
      lm.id = -1; // Unknown ID until matched
      
      // Transform point from lidar frame to map frame
      geometry_msgs::msg::PointStamped point_in_lidar_frame;
      geometry_msgs::msg::PointStamped point_in_map_frame;
      
      point_in_lidar_frame.header.frame_id = lidar_frame_;
      point_in_lidar_frame.header.stamp = msg->header.stamp;
      point_in_lidar_frame.point.x = detected_posts[i].position.x;
      point_in_lidar_frame.point.y = detected_posts[i].position.y;
      point_in_lidar_frame.point.z = 0.0; // Assuming 2D points
      
      // Transform the point
      tf2::doTransform(point_in_lidar_frame, point_in_map_frame, transform);
      
      // Assign transformed coordinates
      lm.x = point_in_map_frame.point.x;
      lm.y = point_in_map_frame.point.y;
      lm.translation_weight = detected_posts[i].translation_weight;
      lm.rotation_weight = detected_posts[i].rotation_weight;
      detected_landmarks.push_back(lm);
    }
    
    // Match detected landmarks with prior map
    auto matched_ids = landmark_matcher_->matchLandmarks(detected_landmarks);

    // Prepare matched landmarks for publishing
    std::vector<Landmark> matched_landmarks;
    std::vector<LandmarkInfo> matched_prior_landmarks;  // 存储匹配的先验地标
    
    for (size_t i = 0; i < detected_landmarks.size(); ++i) {
      if (matched_ids[i] != -1) {
        // Find the corresponding prior landmark
        for (const auto& prior : prior_landmarks_) {
          try {
            if (std::stoi(prior.landmark_id) == matched_ids[i]) {
              matched_landmarks.push_back(detected_landmarks[i]);
              matched_landmarks.back().id = matched_ids[i]; // Set matched ID
              matched_landmarks.back().x = detected_posts[i].position.x; // Set position x in lidar frame
              matched_landmarks.back().y = detected_posts[i].position.y; // Set position y in lidar frame
              matched_prior_landmarks.push_back(prior);  // 保存对应的先验地标
              break;
            }
          } catch (const std::exception& e) {
            continue;
          }
        }
      }
    }
    
    // Publish matched landmarks
    if (!matched_landmarks.empty()) {
      publishLandmarks(matched_landmarks, msg->header.stamp);
      
      if (publish_visualization_) {
        publishVisualizationMarkers(matched_landmarks, msg->header.stamp);
      }
      
      // 计算并发布小车位姿
      geometry_msgs::msg::PoseStamped robot_pose;
      if (calculateRobotPose(matched_prior_landmarks, matched_landmarks, robot_pose)) {
        robot_pose.header.stamp = msg->header.stamp;
        robot_pose.header.frame_id = map_frame_;
        pose_pub_->publish(robot_pose);
        RCLCPP_INFO(this->get_logger(), "Published robot pose in map frame");
      }
      
      RCLCPP_INFO(this->get_logger(), "Published %zu matched landmarks", matched_landmarks.size());
    }
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform error: %s", ex.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing laser scan: %s", e.what());
  }
}

void LandmarkLocalizationNode::publishLandmarks(const std::vector<Landmark>& matched_landmarks,
                                               const builtin_interfaces::msg::Time& stamp) {
  cartographer_ros_msgs::msg::LandmarkList landmark_list;
  landmark_list.header.stamp = stamp;
  landmark_list.header.frame_id = lidar_frame_;
  
  for (const auto& landmark : matched_landmarks) {
    cartographer_ros_msgs::msg::LandmarkEntry entry;
    entry.id = std::to_string(landmark.id);
    
    // Set transform (identity since we're publishing in lidar frame)
    entry.tracking_from_landmark_transform.position.x = landmark.x;
    entry.tracking_from_landmark_transform.position.y = landmark.y;
    entry.tracking_from_landmark_transform.position.z = 0.0;
    entry.tracking_from_landmark_transform.orientation.w = 1.0;
    entry.tracking_from_landmark_transform.orientation.x = 0.0;
    entry.tracking_from_landmark_transform.orientation.y = 0.0;
    entry.tracking_from_landmark_transform.orientation.z = 0.0;
    
    // Set weights
    entry.translation_weight = landmark.translation_weight;
    entry.rotation_weight = landmark.rotation_weight;
    
    landmark_list.landmarks.push_back(entry);
  }
  
  landmark_pub_->publish(landmark_list);
}

void LandmarkLocalizationNode::publishVisualizationMarkers(const std::vector<Landmark>& landmarks,
                                                          const builtin_interfaces::msg::Time& stamp) {
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Create marker for each landmark
  for (size_t i = 0; i < landmarks.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = lidar_frame_;
    marker.ns = "landmarks";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = landmarks[i].x;
    marker.pose.position.y = landmarks[i].y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.8;
    
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = rclcpp::Duration(1s);
    
    marker_array.markers.push_back(marker);
  }
  
  marker_pub_->publish(marker_array);
}

bool LandmarkLocalizationNode::calculateRobotPose(const std::vector<LandmarkInfo>& prior_landmarks,
                                                 const std::vector<Landmark>& detected_landmarks,
                                                 geometry_msgs::msg::PoseStamped& robot_pose) {
  if (prior_landmarks.size() < min_landmarks_for_pose_ || detected_landmarks.size() < min_landmarks_for_pose_) {
    RCLCPP_WARN(this->get_logger(), "Need at least %d matched landmarks for accurate pose calculation, got %zu", 
              min_landmarks_for_pose_, detected_landmarks.size());
    return false;
  }
  
  try {
    // 使用SVD分解计算最优刚体变换
    // 计算两个点集的质心
    Eigen::Vector2d centroid_prior(0, 0);
    Eigen::Vector2d centroid_detected(0, 0);
    
    for (size_t i = 0; i < prior_landmarks.size(); ++i) {
      centroid_prior += Eigen::Vector2d(prior_landmarks[i].x, prior_landmarks[i].y);
      centroid_detected += Eigen::Vector2d(detected_landmarks[i].x, detected_landmarks[i].y);
    }
    centroid_prior /= prior_landmarks.size();
    centroid_detected /= detected_landmarks.size();
    
    // 构建去质心坐标
    Eigen::MatrixXd X(2, prior_landmarks.size());
    Eigen::MatrixXd Y(2, detected_landmarks.size());
    
    for (size_t i = 0; i < prior_landmarks.size(); ++i) {
      X.col(i) = Eigen::Vector2d(prior_landmarks[i].x, prior_landmarks[i].y) - centroid_prior;
      Y.col(i) = Eigen::Vector2d(detected_landmarks[i].x, detected_landmarks[i].y) - centroid_detected;
    }
    
    // 计算协方差矩阵
    Eigen::Matrix2d H = Y * X.transpose();
    
    // SVD分解
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    
    // 计算旋转矩阵 R = V * U^T
    Eigen::Matrix2d R = V * U.transpose();
    
    // 确保右手坐标系（det(R) = 1）
    if (R.determinant() < 0) {
      V.col(1) *= -1;
      R = V * U.transpose();
    }
    
    // 计算平移向量
    Eigen::Vector2d t = centroid_prior - R * centroid_detected;
    
    // 从旋转矩阵提取角度
    double theta = atan2(R(1, 0), R(0, 0));
    
    // 现在得到的是激光雷达在map系下的位姿
    tf2::Transform lidar_to_map_tf;
    lidar_to_map_tf.setOrigin(tf2::Vector3(t.x(), t.y(), 0.0));
    
    tf2::Quaternion lidar_q;
    lidar_q.setRPY(0, 0, theta);
    lidar_to_map_tf.setRotation(lidar_q);
    
    // 如果初始化时没有获取到TF变换，尝试在回调中获取
    tf2::Transform base_to_lidar_tf;
    if (has_base_to_lidar_tf_) {
      base_to_lidar_tf = base_to_lidar_tf_;
    } else {
      // 尝试在回调中获取TF变换（备选方案）
      try {
        geometry_msgs::msg::TransformStamped base_to_lidar_tf_msg = tf_buffer_->lookupTransform(
            base_frame_, lidar_frame_, tf2::TimePointZero);
        tf2::fromMsg(base_to_lidar_tf_msg.transform, base_to_lidar_tf);
        RCLCPP_INFO(this->get_logger(), "Successfully got TF transform in callback");
        has_base_to_lidar_tf_ = true;
        base_to_lidar_tf_ = base_to_lidar_tf;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
                   base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
        return false;
      }
    }
    
    // 计算base_link在map系下的位姿: base_pose = lidar_pose * base_to_lidar_tf
    tf2::Transform base_to_map_tf = lidar_to_map_tf * base_to_lidar_tf;
    
    // 转换为Pose消息
    robot_pose.pose.position.x = base_to_map_tf.getOrigin().x();
    robot_pose.pose.position.y = base_to_map_tf.getOrigin().y();
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = base_to_map_tf.getRotation().x();
    robot_pose.pose.orientation.y = base_to_map_tf.getRotation().y();
    robot_pose.pose.orientation.z = base_to_map_tf.getRotation().z();
    robot_pose.pose.orientation.w = base_to_map_tf.getRotation().w();
    
    // 计算yaw角度（从四元数提取）
    tf2::Quaternion q = base_to_map_tf.getRotation();
    double base_yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    
    RCLCPP_INFO(this->get_logger(), "Base link pose: x=%.3f, y=%.3f, theta=%.3f rad", 
               base_to_map_tf.getOrigin().x(), 
               base_to_map_tf.getOrigin().y(), 
               base_yaw);
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error calculating robot pose: %s", e.what());
    return false;
  }
}

} // namespace landmark_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<landmark_localization::LandmarkLocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}