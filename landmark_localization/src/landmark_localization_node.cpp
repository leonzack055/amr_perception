#include "landmark_localization/landmark_localization_node.hpp"
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <Eigen/Dense> 
#include <Eigen/Geometry>

namespace landmark_localization {

using namespace std::chrono_literals;

LandmarkLocalizationNode::LandmarkLocalizationNode()
: Node("landmark_localization_node")
{
  // 获取yaml配置文件路径
  std::string config_file_path = this->declare_parameter<std::string>("config_file", "");
  
  if (config_file_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "config_file parameter is empty, cannot load parameters from yaml");
    rclcpp::shutdown();
    return;
  }
  
  // 从yaml文件加载参数
  loadParametersFromYaml(config_file_path);

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize components
  landmark_reader_ = std::make_shared<LandmarkReader>(pbstream_file_);
  landmark_matcher_ = std::make_shared<LandmarkMatcher>(matching_threshold_);
  post_detector_ = std::make_shared<ReflectivePostDetector>(intensity_threshold_use,max_age_param);

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
  
  initial_pose_sub_ = this->create_subscription<amr_ros_msg::msg::PoseWithTypeStamped>(
    initial_pose_topic_, 10,
    std::bind(&LandmarkLocalizationNode::initialPoseCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to initial pose topic: %s", initial_pose_topic_.c_str());  

  landmark_pub_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
    landmark_topic_, 10);
  
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    landmark_localization_topic_, 10);
  
  if (publish_visualization_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, 10);
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
  if (lidar2_frame_.empty()) {
    lidar2_frame_ = msg->header.frame_id;
  }
  // 存储最新的scan2消息
  if (use_combine_) {
    std::lock_guard<std::mutex> lock(scan2_mutex_);
    latest_scan2_msg_ = msg;
  } 
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
    RCLCPP_INFO(this->get_logger(), "Scan1 detected %zu posts", scan1_detected_posts.size());
    
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
    
    // 检查scan1检测到的地标是否都是同侧的
    bool scan1_all_same_side = false;
    if (scan1_detected_posts.size() > 0) {
      bool has_positive_y = false;
      bool has_negative_y = false;
      
      for (const auto& post : scan1_detected_posts) {
        if (post.position.y > 0.0) {
          has_positive_y = true;
        } else if (post.position.y < 0.0) {
          has_negative_y = true;
        }
      }
      
      // 如果有正有负，说明不同侧；否则都是同侧
      scan1_all_same_side = !(has_positive_y && has_negative_y);
    }
    
    // 检查是否需要结合scan2数据
    // 条件：1) scan1检测到的地标数量不足 或 2) scan1检测到的地标都是同侧的
    bool need_combine_scan2 = false;
    std::string combine_reason = "";
    if (scan1_detected_posts.size() < min_landmarks_for_pose_) {
      need_combine_scan2 = true;
      combine_reason = "insufficient landmarks";
    } else if (scan1_all_same_side) {
      need_combine_scan2 = true;
      combine_reason = "all landmarks on same side";
    }
    
    if (need_combine_scan2 && use_lidar2_) {
      RCLCPP_INFO(this->get_logger(), "Combining scan2 data, reason: %s", combine_reason.c_str());
      std::lock_guard<std::mutex> lock(scan2_mutex_);
      if (latest_scan2_msg_ != nullptr) {
        // 处理scan2数据（变换到scan1坐标系）
        std::vector<Landmark> scan2_transformed_landmarks;
        if (transformScan2LandmarksToScan1Frame(latest_scan2_msg_, scan1_msg->header.stamp, 
                                              scan1_msg->header.frame_id, scan2_transformed_landmarks)) {
          all_detected_landmarks.insert(all_detected_landmarks.end(), 
                                      scan2_transformed_landmarks.begin(), 
                                      scan2_transformed_landmarks.end());
        }

        RCLCPP_INFO(this->get_logger(), "Total landmarks after combination: %zu", all_detected_landmarks.size());
        
        if (all_detected_landmarks.size() < min_landmarks_for_pose_) {
          RCLCPP_WARN(this->get_logger(), "Combined landmarks still less than %d, skipping pose calculation", min_landmarks_for_pose_);
          return;
        }
      }
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
      }
    }
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform error in combined processing: %s", ex.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing combined scan: %s", e.what());
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
    RCLCPP_INFO(this->get_logger(), "Scan2 detected %zu posts", scan2_detected_posts.size());
    
    if (scan2_detected_posts.empty()) {
      return false;
    }
    
    // 直接查询完整变换：从scan2_frame在scan2时间到target_frame在目标时间
    tf2::Transform tf_complete;
    try {
      geometry_msgs::msg::TransformStamped complete_transform = tf_buffer_->lookupTransform(
        target_frame, target_time,           // 目标坐标系和时间
        scan2_msg->header.frame_id, scan2_msg->header.stamp,  // 源坐标系和时间
        odom_frame_,                          // 固定坐标系（用于时间插值）
        tf2::durationFromSec(tf_time_tolerance_));
      tf2::fromMsg(complete_transform.transform, tf_complete);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
                   target_frame.c_str(), scan2_msg->header.frame_id.c_str(), ex.what());
      return false;
    }
    
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
    return true;
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform error in scan2 landmark transformation: %s", ex.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error transforming scan2 landmarks: %s", e.what());
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
      RCLCPP_INFO(this->get_logger(), "未检测到有效反光柱");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "检测到 %zu 个反光柱", detected_posts.size());
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
      }
    }
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform error for %s: %s", lidar_frame.c_str(), ex.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing laser scan from %s: %s", lidar_frame.c_str(), e.what());
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
  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  if (prior_landmarks.size() < min_landmarks_for_pose_ || detected_landmarks.size() < min_landmarks_for_pose_) {
    RCLCPP_WARN(this->get_logger(), "Need at least %d matched landmarks for accurate pose calculation, got %zu", 
                min_landmarks_for_pose_, detected_landmarks.size());
    return false;
  }

  // 创建可修改的副本
  std::vector<LandmarkInfo> selected_prior_landmarks = prior_landmarks;
  std::vector<Landmark> selected_detected_landmarks = detected_landmarks;

  // 如果检测到的地标数量超过最小值，根据数量选择不同数量的最近地标
  if (detected_landmarks.size() >= min_landmarks_for_pose_) {
    // 计算每个地标的欧氏距离
    std::vector<std::pair<double, size_t>> distances_with_indices;
    for (size_t i = 0; i < detected_landmarks.size(); ++i) {
      double distance = std::sqrt(detected_landmarks[i].x * detected_landmarks[i].x + 
                                 detected_landmarks[i].y * detected_landmarks[i].y);
      distances_with_indices.emplace_back(distance, i);
    }
    
    // 按距离升序排序
    std::sort(distances_with_indices.begin(), distances_with_indices.end(),
              [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                return a.first < b.first;
              });
    
    // 根据检测到的地标数量决定选择多少个地标
    size_t num_to_select;
    if (detected_landmarks.size() == min_landmarks_for_pose_) {
      num_to_select = min_landmarks_for_pose_;
    } else {
      num_to_select = min_landmarks_for_pose_ + 1;
    }
    
    // 选择前num_to_select个最近的地标
    std::vector<LandmarkInfo> temp_prior_landmarks;
    std::vector<Landmark> temp_detected_landmarks;
    std::vector<size_t> selected_indices; // 记录选中的索引
    
    for (size_t i = 0; i < num_to_select; ++i) {
      size_t original_index = distances_with_indices[i].second;
      temp_prior_landmarks.push_back(prior_landmarks[original_index]);
      temp_detected_landmarks.push_back(detected_landmarks[original_index]);
      selected_indices.push_back(original_index);
    }
    
    // 检查选中的地标是否都是同侧的（y值同号）
    bool all_same_side = true;
    bool has_positive_y = false;
    bool has_negative_y = false;
    
    for (const auto& landmark : temp_detected_landmarks) {
      if (landmark.y > 0.0) {
        has_positive_y = true;
      } else if (landmark.y < 0.0) {
        has_negative_y = true;
      }
    }
    
    // 如果有正有负，说明不同侧
    if (has_positive_y && has_negative_y) {
      all_same_side = false;
    }
    
    // 如果选中的都是同侧地标，尝试替换为不同侧的
    if (all_same_side && num_to_select < distances_with_indices.size()) {
      // 确定当前选中的是正侧还是负侧
      bool current_side_is_positive = has_positive_y;
      
      // 在剩余未选中的地标中，找到最近的不同侧地标
      size_t replacement_index = std::numeric_limits<size_t>::max();
      
      for (size_t i = num_to_select; i < distances_with_indices.size(); ++i) {
        size_t candidate_index = distances_with_indices[i].second;
        double candidate_y = detected_landmarks[candidate_index].y;
        
        // 检查是否是不同侧
        bool is_different_side = false;
        if (current_side_is_positive && candidate_y < 0.0) {
          is_different_side = true;
        } else if (!current_side_is_positive && candidate_y > 0.0) {
          is_different_side = true;
        }
        
        if (is_different_side) {
          // 找到第一个不同侧的地标，由于distances_with_indices已按距离排序，第一个不同侧的地标就是最近的
          replacement_index = candidate_index;
          break;
        }
      }
      
      // 如果找到了不同侧的地标，替换选中最远的
      if (replacement_index != std::numeric_limits<size_t>::max()) {
        // 找到选中最远的那个（即最后一个，因为已经按距离排序）
        size_t farthest_in_selected = selected_indices.size() - 1;
        size_t farthest_original_index = selected_indices[farthest_in_selected];
        
        // 替换
        temp_prior_landmarks[farthest_in_selected] = prior_landmarks[replacement_index];
        temp_detected_landmarks[farthest_in_selected] = detected_landmarks[replacement_index];
        selected_indices[farthest_in_selected] = replacement_index;
        
        RCLCPP_INFO(this->get_logger(), "Replaced farthest landmark (ID %d) with different-side landmark (ID %d)", 
                    detected_landmarks[farthest_original_index].id, detected_landmarks[replacement_index].id);
      } else {
        RCLCPP_INFO(this->get_logger(), "All selected landmarks are on the same side, but no different-side landmark found");
      }
    }
    
    // 更新选中的地标
    selected_prior_landmarks = std::move(temp_prior_landmarks);
    selected_detected_landmarks = std::move(temp_detected_landmarks);
    
    RCLCPP_INFO(this->get_logger(), "Selected %zu landmarks from %zu matched landmarks", 
                num_to_select, detected_landmarks.size());
  }

  std::string landmark_ids_str = "";
  for (const auto& landmark : selected_prior_landmarks) {
    if (!landmark_ids_str.empty()) {
      landmark_ids_str += " ";
    }
    landmark_ids_str += landmark.landmark_id;
  }
  RCLCPP_INFO(this->get_logger(), "Selected closest landmarks id: %s", landmark_ids_str.c_str());

  // 新增滤波检查
  if (!checkFilterCondition(selected_prior_landmarks)) {
    return false;
  }

  try {
    // 使用SVD分解计算最优刚体变换
    // 计算两个点集的质心
    Eigen::Vector2d centroid_prior(0, 0);
    Eigen::Vector2d centroid_detected(0, 0);
    
    for (size_t i = 0; i < selected_prior_landmarks.size(); ++i) {
      centroid_prior += Eigen::Vector2d(selected_prior_landmarks[i].x, selected_prior_landmarks[i].y);
      centroid_detected += Eigen::Vector2d(selected_detected_landmarks[i].x, selected_detected_landmarks[i].y);
    }
    centroid_prior /= selected_prior_landmarks.size();
    centroid_detected /= selected_detected_landmarks.size();
    
    // 构建去质心坐标
    Eigen::MatrixXd X(2, selected_prior_landmarks.size());
    Eigen::MatrixXd Y(2, selected_detected_landmarks.size());
    
    for (size_t i = 0; i < selected_prior_landmarks.size(); ++i) {
      X.col(i) = Eigen::Vector2d(selected_prior_landmarks[i].x, selected_prior_landmarks[i].y) - centroid_prior;
      Y.col(i) = Eigen::Vector2d(selected_detected_landmarks[i].x, selected_detected_landmarks[i].y) - centroid_detected;
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
            RCLCPP_DEBUG(this->get_logger(), "Successfully got TF transform in callback");
            has_base_to_lidar1_tf_ = true;
            base_to_lidar1_tf_ = base_to_lidar_tf;
          } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
                        lidar_frame.c_str(), base_frame_.c_str(), ex.what());
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
            RCLCPP_DEBUG(this->get_logger(), "Successfully got TF transform in callback");
            has_base_to_lidar2_tf_ = true;
            base_to_lidar2_tf_ = base_to_lidar_tf;
          } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
                        lidar_frame.c_str(), base_frame_.c_str(), ex.what());
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
    
    RCLCPP_INFO(this->get_logger(), "解算位姿: x=%.3f, y=%.3f, theta=%.3f rad", 
                base_to_map_tf.getOrigin().x(), base_to_map_tf.getOrigin().y(), base_yaw);
    
    // 结束计时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "解算耗时: %ld 毫秒", duration.count());
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error calculating robot pose: %s", e.what());
    return false;
  }
}

void LandmarkLocalizationNode::initialPoseCallback(const amr_ros_msg::msg::PoseWithTypeStamped::SharedPtr msg) {
  try {
    RCLCPP_INFO(this->get_logger(), "Received initial pose message with type: %s", msg->type.c_str());
    
    if (msg->type == "M") {  // 手动模式
      RCLCPP_INFO(this->get_logger(), "Manual mode detected, reloading prior landmarks...");
      
      // 重新加载先验地标
      loadPriorLandmarks();
      
      RCLCPP_INFO(this->get_logger(), "Prior landmarks reloaded successfully");
      
    } else if (msg->type == "A") {  // 自动模式
      RCLCPP_INFO(this->get_logger(), "Auto mode detected, no action taken for landmark reloading");
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown initial pose type: %s", msg->type.c_str());
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing initial pose message: %s", e.what());
  }
}

bool LandmarkLocalizationNode::checkFilterCondition(const std::vector<LandmarkInfo>& selected_prior_landmarks) {
  if (!use_calculate_filter_) {
    return true; // 滤波未启用，直接返回true
  }
  
  std::set<std::string> current_set;
  for (const auto& landmark : selected_prior_landmarks) {
    current_set.insert(landmark.landmark_id);
  }
  
  // 如果当前集合与上一次接受的集合相同，增加计数
  if (current_set == last_accepted_set_) {
    consecutive_count_++;
    if (consecutive_count_ >= std::numeric_limits<int>::max()) {
      consecutive_count_ = filter_num_;
    }
  } else {
    // 检查是否是包含关系
    bool is_subset_relation = false;
    if (!last_accepted_set_.empty()) {
      // 检查 current_set 是否包含 last_accepted_set_
      bool last_is_subset_of_current = std::includes(current_set.begin(), current_set.end(),
                                                      last_accepted_set_.begin(), last_accepted_set_.end());
      // 检查 last_accepted_set_ 是否包含 current_set
      bool current_is_subset_of_last = std::includes(last_accepted_set_.begin(), last_accepted_set_.end(),
                                                      current_set.begin(), current_set.end());
      
      is_subset_relation = last_is_subset_of_current || current_is_subset_of_last;
    }
    
    if (is_subset_relation) {
      // 如果是包含关系，认为无需滤波，继续计数并更新集合
      consecutive_count_++;
      if (consecutive_count_ >= std::numeric_limits<int>::max()) {
        consecutive_count_ = filter_num_;
      }
      last_accepted_set_ = current_set;
    } else {
      // 否则重置计数并更新集合
      consecutive_count_ = 1;
      last_accepted_set_ = current_set;
    }
  }
  
  // 检查是否达到连续k次
  if (consecutive_count_ >= filter_num_) {
    RCLCPP_DEBUG(this->get_logger(), "Filter condition satisfied: consecutive count %d >= filter_num=%d", 
                 consecutive_count_, filter_num_);
    return true;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Filter condition not satisfied: current consecutive count %d < filter_num=%d", 
                 consecutive_count_, filter_num_);
    
    return false;
  }
}

void LandmarkLocalizationNode::loadParametersFromYaml(const std::string& yaml_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_path);
    
    if (!config["landmark_localization"]) {
      RCLCPP_ERROR(this->get_logger(), "YAML file does not contain 'landmark_localization' key");
      return;
    }
    
    YAML::Node params = config["landmark_localization"];
    
    pbstream_file_ = params["pbstream_file"].as<std::string>();
    map_frame_ = params["map_frame"].as<std::string>();
    base_frame_ = params["base_frame"].as<std::string>();
    odom_frame_ = params["odom_frame"].as<std::string>();
    matching_threshold_ = params["matching_threshold"].as<double>();
    publish_visualization_ = params["publish_visualization"].as<bool>();
    intensity_threshold_use = params["intensity_threshold_use"].as<int>();
    max_age_param = params["max_age_param"].as<int>();
    tf_time_tolerance_ = params["tf_time_tolerance"].as<double>();
    min_landmarks_for_pose_ = params["min_landmarks_for_pose"].as<int>();
    use_calculate_filter_ = params["use_calculate_filter"].as<bool>();
    filter_num_ = params["filter_num"].as<int>();
    use_lidar1_ = params["use_lidar1"].as<bool>();
    scan1_topic_ = params["scan1_topic"].as<std::string>();
    use_lidar2_ = params["use_lidar2"].as<bool>();
    scan2_topic_ = params["scan2_topic"].as<std::string>();
    use_combine_ = params["use_combine"].as<bool>();
    if (!use_lidar2_ && use_combine_) {
      use_combine_ = false;
      RCLCPP_WARN(this->get_logger(), "Combined processing is disabled since lidar2 is not used");
    }
    landmark_topic_ = params["landmark_topic"].as<std::string>();
    visualization_topic_ = params["visualization_topic"].as<std::string>();
    landmark_localization_topic_ = params["landmark_localization_topic"].as<std::string>();
    initial_pose_topic_ = params["initial_pose_topic"].as<std::string>();
    
    RCLCPP_INFO(this->get_logger(), "Successfully loaded parameters from yaml file: %s", yaml_file_path.c_str());
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file %s: %s", yaml_file_path.c_str(), e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading YAML file %s: %s", yaml_file_path.c_str(), e.what());
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