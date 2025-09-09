#include "landmark_localization/reflective_post_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace landmark_localization {

ReflectivePostDetector::ReflectivePostDetector(bool use_simulation_params)
{
  if (use_simulation_params) {
    intensity_threshold_use = 200;
    cluster_eps = 0.04;
    min_cluster_points = 10;
    diameter_min = 0.05;
    diameter_max = 0.12;
    residual_avg_threshold = 0.01;
    residual_std_threshold = 0.005;
    residual_max_threshold = 0.02;
    stat_mean_k = 5;
    stat_std_threshold = 1.0;
    max_history_age = 3;
    match_distance_threshold = 0.1;
    arc_threshold = 0.1;
    max_arc_feature = 30.0;
    arc_min_points = 8;
    direction_tolerance = 0.785;
    residual_real = 0.032;
    sensitivity = 2.0;
    maxError = 1.0;
    maxangleError = 0.1;
  } else {
    intensity_threshold_use = 1000;
    cluster_eps = 0.04;
    min_cluster_points = 10;
    diameter_min = 0.05;
    diameter_max = 0.08;
    residual_avg_threshold = 0.01;
    residual_std_threshold = 0.005;
    residual_max_threshold = 0.02;
    stat_mean_k = 5;
    stat_std_threshold = 1.0;
    max_history_age = 3;
    match_distance_threshold = 0.1;
    arc_threshold = 0.1;
    max_arc_feature = 8.0;
    arc_min_points = 8;
    direction_tolerance = 0.785;
    residual_real = 0.032;
    sensitivity = 2.0;
    maxError = 1.0;
    maxangleError = 0.1;
  }
}

ReflectivePostDetector::~ReflectivePostDetector()
{
}

std::vector<ReflectivePost> ReflectivePostDetector::detect(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::vector<ReflectivePost> detected_posts;

  // 0903 xusha begin
  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int intensity_thresh = intensity_threshold_use;
  //RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "实际使用的强度阈值: %d", intensity_thresh);

  // 提取高强度点
  std::vector<Vector2d> points;
  std::vector<float> high_intensities;
  
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (msg->intensities[i] > intensity_thresh &&
          msg->ranges[i] > msg->range_min &&
          msg->ranges[i] < msg->range_max &&
          !std::isnan(msg->ranges[i])) {
          
          double angle = msg->angle_min + i * msg->angle_increment;
          Vector2d tmp_p;
          tmp_p.x = msg->ranges[i] * std::cos(angle);
          tmp_p.y = msg->ranges[i] * std::sin(angle);
          points.push_back(tmp_p);
          high_intensities.push_back(msg->intensities[i]);
      }
  }

  //RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "通过强度筛选的点数量: %zu", points.size());
  if (points.empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("ReflectivePostDetector"), "未检测到高强度点，不发布结果");
      return detected_posts;
  }

  // 去噪
  if (points.size() > 10) {
      points = statistical_outlier_filter(points);
  }

  // 聚类
  std::vector<int> labels;
  if (int(points.size()) >= min_cluster_points) {
      labels = adaptive_dbscan(points);
      //RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "info lable size: %d", int(labels.size()));
  } else {
      labels = std::vector<int>(points.size(), -1);
  }
      
  // 提取有效聚类点
  std::vector<Vector2d> valid_points;
  std::vector<int> valid_labels;
  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] != -1) {
        valid_points.push_back(points[i]);
        valid_labels.push_back(labels[i]);
    }
  }

  if (valid_points.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "未检测到有效聚类，不发布结果");
    return detected_posts;
  }
  std::vector<int> unique_labels = valid_labels;
  std::sort(unique_labels.begin(), unique_labels.end());
  unique_labels.erase(std::unique(unique_labels.begin(), unique_labels.end()), unique_labels.end());

  std::vector<Detection> current_detections;
  for (int label : unique_labels) {
    std::vector<Vector2d> cluster;
    for (size_t i = 0; i < valid_labels.size(); ++i) {
      if (valid_labels[i] == label) {
          cluster.push_back(valid_points[i]);
      }
    }
    CircleFitResult circle = circle_fit(cluster);
    if (circle.valid && circle.r_squared >= 0.85){
      double diameter = 2 * circle.radius;
      if (diameter > diameter_min && diameter < diameter_max){
        double arc_feature = calculate_arc_feature(cluster);
        double max_arc = max_arc_feature;
        //RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "info arc_feature: %f",arc_feature);
        if (circle.avg_residual < residual_avg_threshold &&
            circle.std_residual < residual_std_threshold &&
            circle.max_residual < residual_max_threshold &&
            arc_feature > arc_threshold && arc_feature < max_arc) {
              double x_ = -circle.center.x;
              double y_ = -circle.center.y;
              double normal_theta = std::atan2(y_,x_);
              if(normal_theta > M_PI){
                normal_theta = 2*M_PI - normal_theta;
              }
              Vector2d real_p = circle_real(cluster);
              double t_w = calculateTranslationWeight(circle.center,real_p);
              double r_w = calculateRotationWeight(circle.center,real_p);
              geometry_msgs::msg::PoseStamped pose;
              pose.header = msg->header;
              pose.pose.position.x = circle.center.x;
              pose.pose.position.y = circle.center.y;
              pose.pose.orientation.z = std::sin(normal_theta / 2);
              pose.pose.orientation.w = std::cos(normal_theta / 2);

              // 计算置信度
              double confidence = std::min(1.0, 0.8 + 0.2 * (points.size() / 20.0)) * 
                                  std::min(1.0, circle.r_squared) * 
                                  std::min(1.0, arc_feature / arc_threshold);

              Detection tmp_results;
              tmp_results.pose = pose;
              tmp_results.diameter = diameter;
              tmp_results.confidence = confidence;
              tmp_results.translationW = t_w;
              tmp_results.rotationW = r_w;
              current_detections.push_back(tmp_results);
        }
      }
    }
  }
  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "检测耗时: " << duration.count() << " 毫秒" << std::endl;

  for (const auto& detection : current_detections) {
      ReflectivePost post;
      post.position.x = detection.pose.pose.position.x;
      post.position.y = detection.pose.pose.position.y;
      post.position.z = 0.0;
      post.translation_weight = detection.translationW;
      post.rotation_weight = detection.rotationW;
      detected_posts.push_back(post);

      std::cout << "反光柱检测结果：圆心(" << post.position.x << "m, " << post.position.y << "m), t_w=" 
                << post.translation_weight << ", r_w=" << post.rotation_weight << std::endl;
  }


  // 0903 xusha end
  
  // // Create 3 fake reflective posts for testing
  // RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), 
  //             "Detecting reflective posts (fake data for testing)");

  // ReflectivePost post1;
  // post1.position.x = 1.0;
  // post1.position.y = 2.0;
  // post1.position.z = 0.0;
  // detected_posts.push_back(post1);
  
  // ReflectivePost post2;
  // post2.position.x = 3.0;
  // post2.position.y = 4.0;
  // post2.position.z = 0.0;
  // detected_posts.push_back(post2);
  
  // ReflectivePost post3;
  // post3.position.x = 3.0;
  // post3.position.y = 2.0;
  // post3.position.z = 0.0;
  // detected_posts.push_back(post3);  
  return detected_posts;
}

} // namespace landmark_localization