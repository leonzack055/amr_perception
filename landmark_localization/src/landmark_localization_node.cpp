#include "landmark_localization/landmark_localization_node.hpp"
#include "landmark_localization/landmark_reader.hpp"
#include "landmark_localization/landmark_matcher.hpp"
#include "landmark_localization/reflective_post_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <memory>
#include <mutex>

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
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("matching_threshold", 0.5);
  this->declare_parameter("publish_visualization", true);
  this->declare_parameter("intensity_threshold_use", 1600);
  this->declare_parameter("tf_time_tolerance", 0.05);
  this->declare_parameter("min_landmarks_for_pose", 3);
  
  // 激光雷达1参数
  this->declare_parameter("use_lidar1", true);
  this->declare_parameter("lidar1_frame", "laser_1");
  this->declare_parameter("scan1_topic", "/scan_1");
  
  // 激光雷达2参数
  this->declare_parameter("use_lidar2", true);
  this->declare_parameter("lidar2_frame", "laser_2");
  this->declare_parameter("scan2_topic", "/scan_2");

  this->declare_parameter("use_combine", true);
  
  this->declare_parameter("landmark_topic", "/landmark");
  this->declare_parameter("visualization_topic", "/landmark_localization_markers");
  this->declare_parameter("landmark_localization_topic", "/global_pose_qr");
  
  // Get parameters
  pbstream_file_ = this->get_parameter("pbstream_file").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  matching_threshold_ = this->get_parameter("matching_threshold").as_double();
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  intensity_threshold_use = this->get_parameter("intensity_threshold_use").as_int();
  tf_time_tolerance_ = this->get_parameter("tf_time_tolerance").as_double();
  min_landmarks_for_pose_ = this->get_parameter("min_landmarks_for_pose").as_int();
  
  // 获取激光雷达1参数
  use_lidar1_ = this->get_parameter("use_lidar1").as_bool();
  scan1_topic_ = this->get_parameter("scan1_topic").as_string();
  
  // 获取激光雷达2参数
  use_lidar2_ = this->get_parameter("use_lidar2").as_bool();
  scan2_topic_ = this->get_parameter("scan2_topic").as_string();

  use_combine_ = this->get_parameter("use_combine").as_bool();
  if (!use_lidar2_ && use_combine_) {
    use_combine_ = false;
    RCLCPP_WARN(this->get_logger(), "Combined processing is disabled since lidar2 is not used");
  }
  
  std::string landmark_topic = this->get_parameter("landmark_topic").as_string();
  std::string visualization_topic = this->get_parameter("visualization_topic").as_string();
  std::string landmark_localization_topic = this->get_parameter("landmark_localization_topic").as_string();

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize components
  landmark_reader_ = std::make_shared<LandmarkReader>(pbstream_file_);
  landmark_matcher_ = std::make_shared<LandmarkMatcher>(matching_threshold_);
  post_detector_ = std::make_shared<ReflectivePostDetector>(intensity_threshold_use);

  // Load prior landmarks from map
  loadPriorLandmarks();
  
  // Create subscribers and publishers
  if (use_lidar1_) {
    scan1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan1_topic_, 10,
      std::bind(&LandmarkLocalizationNode::laserScan1Callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to lidar1 topic: %s", scan1_topic_.c_str());
  }
  
  if (use_lidar2_) {
    scan2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan2_topic_, 10,
      std::bind(&LandmarkLocalizationNode::laserScan2Callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to lidar2 topic: %s", scan2_topic_.c_str());
  }
  
  landmark_pub_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
    landmark_topic, 10);
  
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    landmark_localization_topic, 10);
  
  if (publish_visualization_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic, 10);
  }
  
  RCLCPP_INFO(this->get_logger(), "Landmark localization node started");
  RCLCPP_INFO(this->get_logger(), "Using lidar1: %s, lidar2: %s", 
             use_lidar1_ ? "true" : "false", use_lidar2_ ? "true" : "false");
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

void LandmarkLocalizationNode::laserScan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::cout << "---------------scan1---------------" << std::endl;
  if (lidar1_frame_.empty()) {
    lidar1_frame_ = msg->header.frame_id;
  }
  if (use_combine_) {
    processCombinedScan(msg);
  } else {
    processLaserScan(msg);
  }
}

void LandmarkLocalizationNode::laserScan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::cout << "---------------scan2---------------" << std::endl;
  if (lidar2_frame_.empty()) {
    lidar2_frame_ = msg->header.frame_id;
  }
  // 存储最新的scan2消息
  if (use_combine_) {
    std::lock_guard<std::mutex> lock(scan2_mutex_);
    latest_scan2_msg_ = msg;
  } 
  processLaserScan(msg);
}

void LandmarkLocalizationNode::processCombinedScan(const sensor_msgs::msg::LaserScan::SharedPtr scan1_msg) {
  try {
    std::vector<Landmark> all_detected_landmarks;
    std::string lidar_frame = scan1_msg->header.frame_id;
    
    // 处理scan1数据
    LaserScan scan1;
    scan1.ranges = scan1_msg->ranges;
    scan1.intensities = scan1_msg->intensities;
    scan1.angle_min = scan1_msg->angle_min;
    scan1.angle_max = scan1_msg->angle_max;
    scan1.angle_increment = scan1_msg->angle_increment;
    scan1.scan_time = scan1_msg->scan_time;
    scan1.range_min = scan1_msg->range_min;
    scan1.range_max = scan1_msg->range_max;
    
    auto scan1_detected_posts = post_detector_->detect(scan1);
    std::cout << "Scan1 detected " << scan1_detected_posts.size() << " posts" << std::endl;
    
    // 将scan1检测到的反光柱转换为Landmark格式（在scan1坐标系下）
    for (size_t i = 0; i < scan1_detected_posts.size(); ++i) {
      Landmark lm;
      lm.id = -1;
      lm.x = scan1_detected_posts[i].position.x;
      lm.y = scan1_detected_posts[i].position.y;
      lm.translation_weight = scan1_detected_posts[i].translation_weight;
      lm.rotation_weight = scan1_detected_posts[i].rotation_weight;
      all_detected_landmarks.push_back(lm);
    }
    
    // 检查是否需要结合scan2数据
    if (scan1_detected_posts.size() < min_landmarks_for_pose_ && use_lidar2_) {
      std::lock_guard<std::mutex> lock(scan2_mutex_);
      if (latest_scan2_msg_ != nullptr) {
        std::cout << "Scan1 detected " << scan1_detected_posts.size() << " posts, combining with scan2 data" << std::endl;
        // 处理scan2数据（变换到scan1坐标系）
        std::vector<Landmark> scan2_transformed_landmarks;
        if (transformScan2LandmarksToScan1Frame(latest_scan2_msg_, scan1_msg->header.stamp, 
                                              scan1_msg->header.frame_id, scan2_transformed_landmarks)) {
          std::cout << "Successfully transformed " << scan2_transformed_landmarks.size() << " landmarks from scan2 to scan1 frame" << std::endl;
          all_detected_landmarks.insert(all_detected_landmarks.end(), 
                                      scan2_transformed_landmarks.begin(), 
                                      scan2_transformed_landmarks.end());
        }
      }
    }

    std::cout << "Total landmarks after combination: " << all_detected_landmarks.size() << std::endl;
    
    if (all_detected_landmarks.size() < min_landmarks_for_pose_) {
      std::cout << "Combined landmarks still less than " << min_landmarks_for_pose_ << ", skipping pose calculation" << std::endl;
      return;
    }
    
    // 获取从scan1坐标系到map坐标系的变换
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_->lookupTransform(
        map_frame_, 
        lidar_frame, 
        scan1_msg->header.stamp,
        tf2::durationFromSec(tf_time_tolerance_));
    
    // 将检测到的地标变换到map坐标系
    std::vector<Landmark> detected_landmarks_in_map;
    for (const auto& landmark : all_detected_landmarks) {
      Landmark lm = landmark;
      
      // Transform point from lidar frame to map frame
      geometry_msgs::msg::PointStamped point_in_lidar_frame;
      geometry_msgs::msg::PointStamped point_in_map_frame;
      
      point_in_lidar_frame.header.frame_id = lidar_frame;
      point_in_lidar_frame.header.stamp = scan1_msg->header.stamp;
      point_in_lidar_frame.point.x = landmark.x;
      point_in_lidar_frame.point.y = landmark.y;
      point_in_lidar_frame.point.z = 0.0;
      
      // Transform the point
      tf2::doTransform(point_in_lidar_frame, point_in_map_frame, transform);
      
      // Assign transformed coordinates
      lm.x = point_in_map_frame.point.x;
      lm.y = point_in_map_frame.point.y;
      detected_landmarks_in_map.push_back(lm);
    }
    
    // Match detected landmarks with prior map
    auto matched_ids = landmark_matcher_->matchLandmarks(detected_landmarks_in_map);

    // Prepare matched landmarks for publishing
    std::vector<Landmark> matched_landmarks;
    std::vector<LandmarkInfo> matched_prior_landmarks;
    
    for (size_t i = 0; i < detected_landmarks_in_map.size(); ++i) {
      if (matched_ids[i] != -1) {
        for (const auto& prior : prior_landmarks_) {
          try {
            if (std::stoi(prior.landmark_id) == matched_ids[i]) {
              matched_landmarks.push_back(detected_landmarks_in_map[i]);
              matched_landmarks.back().id = matched_ids[i];
              matched_landmarks.back().x = all_detected_landmarks[i].x; // Set position x in lidar frame
              matched_landmarks.back().y = all_detected_landmarks[i].y; // Set position y in lidar frame
              matched_prior_landmarks.push_back(prior);
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
      publishLandmarks(matched_landmarks, scan1_msg->header.stamp, lidar_frame);
      
      if (publish_visualization_) {
        publishVisualizationMarkers(matched_landmarks, scan1_msg->header.stamp, lidar_frame);
      }
      
      // 计算并发布小车位姿
      geometry_msgs::msg::PoseStamped robot_pose;
      if (calculateRobotPose(matched_prior_landmarks, matched_landmarks, robot_pose, lidar_frame)) {
        robot_pose.header.stamp = scan1_msg->header.stamp;
        robot_pose.header.frame_id = map_frame_;
        pose_pub_->publish(robot_pose);
        std::cout << "Published robot pose in map frame from combined scan" << std::endl;
      }
      std::cout << "Published " << matched_landmarks.size() << " matched landmarks from combined scan" << std::endl;
    }
    
  } catch (tf2::TransformException &ex) {
    std::cout << "TF transform error in combined processing: " << ex.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error processing combined scan: " << e.what() << std::endl;
  }
}

bool LandmarkLocalizationNode::transformScan2LandmarksToScan1Frame(
    const sensor_msgs::msg::LaserScan::SharedPtr scan2_msg,
    const builtin_interfaces::msg::Time& target_time,
    const std::string& target_frame,
    std::vector<Landmark>& transformed_landmarks) {
  
  try {
    // 检测scan2中的反光柱
    LaserScan scan2;
    scan2.ranges = scan2_msg->ranges;
    scan2.intensities = scan2_msg->intensities;
    scan2.angle_min = scan2_msg->angle_min;
    scan2.angle_max = scan2_msg->angle_max;
    scan2.angle_increment = scan2_msg->angle_increment;
    scan2.scan_time = scan2_msg->scan_time;
    scan2.range_min = scan2_msg->range_min;
    scan2.range_max = scan2_msg->range_max;
    
    auto scan2_detected_posts = post_detector_->detect(scan2);
    std::cout << "Scan2 detected " << scan2_detected_posts.size() << " posts at original time" << std::endl;
    
    if (scan2_detected_posts.empty()) {
      return false;
    } else if (scan2_detected_posts.size() >= min_landmarks_for_pose_) {
      std::cout << "Scan2 alone has enough landmarks (" << scan2_detected_posts.size() << "), no need to transform" << std::endl;
      return false;
    }
    
    // 第一步：获取从scan2时间到目标时间的scan2_frame自身运动变换
    // 通过odom坐标系作为中介
    
    // 1.1 获取scan2_frame在scan2时间在odom系下的位姿
    geometry_msgs::msg::TransformStamped scan2_to_odom_at_scan2_time;
    scan2_to_odom_at_scan2_time = tf_buffer_->lookupTransform(
        odom_frame_, scan2_msg->header.frame_id, scan2_msg->header.stamp, tf2::durationFromSec(tf_time_tolerance_));
    
    // 1.2 获取scan2_frame在目标时间在odom系下的位姿
    geometry_msgs::msg::TransformStamped scan2_to_odom_at_target_time;
    scan2_to_odom_at_target_time = tf_buffer_->lookupTransform(
        odom_frame_, scan2_msg->header.frame_id, target_time, tf2::durationFromSec(tf_time_tolerance_));
    
    // 计算scan2_frame从scan2时间到目标时间的自身运动变换
    // transform1: scan2_time的scan2_frame → target_time的scan2_frame
    tf2::Transform tf_scan2_to_odom_at_scan2_time;
    tf2::fromMsg(scan2_to_odom_at_scan2_time.transform, tf_scan2_to_odom_at_scan2_time);
    
    tf2::Transform tf_scan2_to_odom_at_target_time;
    tf2::fromMsg(scan2_to_odom_at_target_time.transform, tf_scan2_to_odom_at_target_time);
    
    // 从odom系变换回scan2_frame：scan2_frame_target_time = odom_to_scan2_target_time * scan2_to_odom_scan2_time
    tf2::Transform tf_scan2_time_to_target_time = 
        tf_scan2_to_odom_at_target_time.inverse() * tf_scan2_to_odom_at_scan2_time;
    
    // 第二步：获取从scan2_frame到target_frame的静态变换
    tf2::Transform tf_scan2_to_target;
    if (has_lidar2_to_lidar1_tf_) {
      tf_scan2_to_target = lidar2_to_lidar1_tf_;
    } else {
        try {
          geometry_msgs::msg::TransformStamped scan2_to_target = tf_buffer_->lookupTransform(
              target_frame, scan2_msg->header.frame_id, tf2::TimePointZero);
          tf2::fromMsg(scan2_to_target.transform, tf_scan2_to_target);
          std::cout << "Successfully got TF transform in callback" << std::endl;
          has_lidar2_to_lidar1_tf_ = true;
          lidar2_to_lidar1_tf_ = tf_scan2_to_target;
        } catch (tf2::TransformException &ex) {
          std::cerr << "Could not transform " << target_frame << " to " << scan2_msg->header.frame_id << ": " << ex.what() << std::endl;
          return false;
        }
    }
        
    // 第三步：组合变换
    // 完整变换 = 静态TF(scan2→target) × 时间运动变换(scan2_time→scan2_target_time)
    tf2::Transform tf_complete = tf_scan2_to_target * tf_scan2_time_to_target_time;
    
    // 变换scan2检测到的地标
    for (const auto& post : scan2_detected_posts) {
      // 创建scan2时间下scan2坐标系中的点
      tf2::Vector3 point_in_scan2_at_scan2_time(post.position.x, post.position.y, 0.0);
      
      // 应用完整变换：先变换到目标时间的scan2坐标系，再变换到目标坐标系
      tf2::Vector3 point_in_target_at_target_time = tf_complete * point_in_scan2_at_scan2_time;
      
      Landmark lm;
      lm.id = -1;
      lm.x = point_in_target_at_target_time.x();
      lm.y = point_in_target_at_target_time.y();
      lm.translation_weight = post.translation_weight;
      lm.rotation_weight = post.rotation_weight;
      transformed_landmarks.push_back(lm);
    }
    
    std::cout << "Transformed " << scan2_detected_posts.size() << " landmarks from scan2 frame to " << target_frame << " frame" << std::endl;
    return true;
    
  } catch (tf2::TransformException &ex) {
    std::cout << "TF transform error in scan2 landmark transformation: " << ex.what() << std::endl;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "Error transforming scan2 landmarks: " << e.what() << std::endl;
    return false;
  }
}

void LandmarkLocalizationNode::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::string lidar_frame = msg->header.frame_id;
  try {    
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

    // Get transform from lidar to map frame
    geometry_msgs::msg::TransformStamped transform;

    // 允许一定时间范围内的最近变换
    transform = tf_buffer_->lookupTransform(
        map_frame_, 
        lidar_frame, 
        msg->header.stamp,
        tf2::durationFromSec(tf_time_tolerance_));

    // Convert detected posts to Landmark format for matching
    std::vector<Landmark> detected_landmarks;
    for (size_t i = 0; i < detected_posts.size(); ++i) {
      Landmark lm;
      lm.id = -1; // Unknown ID until matched
      
      // Transform point from lidar frame to map frame
      geometry_msgs::msg::PointStamped point_in_lidar_frame;
      geometry_msgs::msg::PointStamped point_in_map_frame;
      
      point_in_lidar_frame.header.frame_id = lidar_frame;
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
      publishLandmarks(matched_landmarks, msg->header.stamp, lidar_frame);
      
      if (publish_visualization_) {
        publishVisualizationMarkers(matched_landmarks, msg->header.stamp, lidar_frame);
      }
      
      // 计算并发布小车位姿
      geometry_msgs::msg::PoseStamped robot_pose;
      if (calculateRobotPose(matched_prior_landmarks, matched_landmarks, robot_pose, lidar_frame)) {
        robot_pose.header.stamp = msg->header.stamp;
        robot_pose.header.frame_id = map_frame_;
        pose_pub_->publish(robot_pose);
        std::cout << "Published robot pose in map frame from " << lidar_frame << std::endl;
      }
      std::cout << "Published " << matched_landmarks.size() << " matched landmarks from " << lidar_frame << std::endl;
    }
    
  } catch (tf2::TransformException &ex) {
    std::cout << "TF transform error for " << lidar_frame << ": " << ex.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error processing laser scan from " << lidar_frame << ": " << e.what() << std::endl;
  }
}

void LandmarkLocalizationNode::publishLandmarks(const std::vector<Landmark>& matched_landmarks,
                                               const builtin_interfaces::msg::Time& stamp,
                                               const std::string& lidar_frame) {
  cartographer_ros_msgs::msg::LandmarkList landmark_list;
  landmark_list.header.stamp = stamp;
  landmark_list.header.frame_id = lidar_frame;  // 使用对应的激光雷达frame
  
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
                                                          const builtin_interfaces::msg::Time& stamp,
                                                          const std::string& lidar_frame) {
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Create marker for each landmark
  for (size_t i = 0; i < landmarks.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = lidar_frame;  // 使用对应的激光雷达frame
    marker.ns = "landmarks_" + lidar_frame;  // 添加frame标识避免冲突
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
                                                 geometry_msgs::msg::PoseStamped& robot_pose,
                                                 const std::string& lidar_frame) {
  if (prior_landmarks.size() < min_landmarks_for_pose_ || detected_landmarks.size() < min_landmarks_for_pose_) {
    std::cerr << "Need at least " << min_landmarks_for_pose_ << " matched landmarks for accurate pose calculation, got " << detected_landmarks.size() << std::endl;
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
    if (lidar_frame == lidar1_frame_) {
      if (has_base_to_lidar1_tf_) {
        base_to_lidar_tf = base_to_lidar1_tf_;
      } else {
          try {
            geometry_msgs::msg::TransformStamped base_to_lidar_tf_msg = tf_buffer_->lookupTransform(
                lidar_frame, base_frame_, tf2::TimePointZero);
            tf2::fromMsg(base_to_lidar_tf_msg.transform, base_to_lidar_tf);
            std::cout << "Successfully got TF transform in callback" << std::endl;
            has_base_to_lidar1_tf_ = true;
            base_to_lidar1_tf_ = base_to_lidar_tf;
          } catch (tf2::TransformException &ex) {
            std::cerr << "Could not transform " << lidar_frame << " to " << base_frame_ << ": " << ex.what() << std::endl;
            return false;
          }
      }
    } else {
      if (has_base_to_lidar2_tf_) {
        base_to_lidar_tf = base_to_lidar2_tf_;
      } else {
          try {
            geometry_msgs::msg::TransformStamped base_to_lidar_tf_msg = tf_buffer_->lookupTransform(
                lidar_frame, base_frame_, tf2::TimePointZero);
            tf2::fromMsg(base_to_lidar_tf_msg.transform, base_to_lidar_tf);
            std::cout << "Successfully got TF transform in callback" << std::endl;
            has_base_to_lidar2_tf_ = true;
            base_to_lidar2_tf_ = base_to_lidar_tf;
          } catch (tf2::TransformException &ex) {
            std::cerr << "Could not transform " << lidar_frame << " to " << base_frame_ << ": " << ex.what() << std::endl;
            return false;
          }
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
    
    std::cout << "Base link pose: x=" << base_to_map_tf.getOrigin().x() << ", y=" << base_to_map_tf.getOrigin().y() << ", theta=" << base_yaw << " rad" << std::endl;
    return true;
    
  } catch (const std::exception& e) {
    std::cerr << "Error calculating robot pose: " << e.what() << std::endl;
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