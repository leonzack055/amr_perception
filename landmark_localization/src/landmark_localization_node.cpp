#include "landmark_localization/landmark_localization_node.hpp"
#include "landmark_localization/landmark_reader.hpp"
#include "landmark_localization/landmark_matcher.hpp"
#include "landmark_localization/reflective_post_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <memory>

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
  this->declare_parameter("scan_topic", "/scan");
  this->declare_parameter("landmark_topic", "/landmark");
  this->declare_parameter("visualization_topic", "/landmark_localization_markers");
  
  // Get parameters
  pbstream_file_ = this->get_parameter("pbstream_file").as_string();
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  matching_threshold_ = this->get_parameter("matching_threshold").as_double();
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
  intensity_threshold_use = this->get_parameter("intensity_threshold_use").as_int();
  tf_time_tolerance_ = this->get_parameter("tf_time_tolerance").as_double();
  std::string scan_topic = this->get_parameter("scan_topic").as_string();
  std::string landmark_topic = this->get_parameter("landmark_topic").as_string();
  std::string visualization_topic = this->get_parameter("visualization_topic").as_string();
  
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
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 10,
    std::bind(&LandmarkLocalizationNode::laserScanCallback, this, std::placeholders::_1));
  
  landmark_pub_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
    landmark_topic, 10);
  
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

} // namespace landmark_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<landmark_localization::LandmarkLocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}