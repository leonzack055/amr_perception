#include "landmark_localization/reflective_post_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace landmark_localization {

ReflectivePostDetector::ReflectivePostDetector(bool use_simulation_params)
{
  if (use_simulation_params) {
    intensity_threshold_use = 200;
  }
}

ReflectivePostDetector::~ReflectivePostDetector()
{
}

std::vector<ReflectivePost> ReflectivePostDetector::detect(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::vector<ReflectivePost> detected_posts;

  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int intensity_thresh = intensity_threshold_use;
  //RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "实际使用的强度阈值: %d", intensity_thresh);

  // 提取高强度点
  std::vector<Vector2d> points;
  std::vector<float> high_intensities;
  std::vector<Vector2d> param_ori;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (msg->intensities[i] > intensity_thresh &&
          msg->ranges[i] > msg->range_min &&
          msg->ranges[i] < msg->range_max &&
        !std::isnan(msg->ranges[i]))
      {
          
          double angle = msg->angle_min + i * msg->angle_increment;
          Vector2d tmp_p, tmp_param;
          tmp_p.x = msg->ranges[i] * std::cos(angle);
          tmp_p.y = msg->ranges[i] * std::sin(angle);
          tmp_param.x = msg->ranges[i];
          tmp_param.y = angle;
          points.push_back(tmp_p);
          high_intensities.push_back(msg->intensities[i]);
          param_ori.push_back(tmp_param);
      }
  }

  if (points.empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("ReflectivePostDetector"), "未检测到高强度点，不发布结果");
      return detected_posts;
  }
  int min_cluster_points_ = min_cluster_points;
  // 去噪
  if (points.size() > min_cluster_points_) {
      OrinParam orin_points = statistical_outlier_filter(points, high_intensities, param_ori);
      points = orin_points.points;
      high_intensities = orin_points.high;
      param_ori = orin_points.param_;
  }
  /*int high_index = findMaxIndexStd(high_intensities);
  RCLCPP_INFO(this->get_logger(), "通过强度筛选的点数量: %zu,%d", points.size(),high_index);*/
  // 聚类
  std::vector<int> labels;
  if (int(points.size()) >= min_cluster_points) {
      labels = adaptive_dbscan(points);
  } else {
      labels = std::vector<int>(points.size(), -1);
  }
  // 提取有效聚类点
  std::vector<Vector2d> valid_points, valid_params;
  std::vector<int> valid_labels;
  std::vector<float> valid_high;
  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] != -1) {
        valid_points.push_back(points[i]);
        valid_labels.push_back(labels[i]);
        valid_high.push_back(high_intensities[i]);
        valid_params.push_back(param_ori[i]);
    }
  }
  if (valid_points.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("ReflectivePostDetector"), "未检测到有效聚类，不发布结果");
    return detected_posts;
  }
  std::vector<int> unique_labels = valid_labels;
  std::sort(unique_labels.begin(), unique_labels.end());
  unique_labels.erase(
    std::unique(unique_labels.begin(), unique_labels.end()),
    unique_labels.end());
  std::vector<Detection> current_detections;
  for (int label : unique_labels) {
    std::vector<Vector2d> cluster, cluster_params;
    std::vector<float> cluster_high;
    for (size_t i = 0; i < valid_labels.size(); ++i) {
      if (valid_labels[i] == label) {
          cluster.push_back(valid_points[i]);
          cluster_high.push_back(valid_high[i]);
          cluster_params.push_back(valid_params[i]);
      }
    }
    int high_index_filter = findMaxIndexStd(cluster_high);
    double percentageNumber = float(high_index_filter) / float(cluster.size());
    std::cout << "占比: " << percentageNumber << std::endl;
    if (percentageNumber >= percentage_min && percentageNumber <= percentage_max) {
      double radius_ = residual_real;
      Vector2d center_points_result = cluster_params[high_index_filter];
      double circle_center_x = (center_points_result.x + radius_) * std::cos(
        center_points_result.y);
      double circle_center_y = (center_points_result.x + radius_) * std::sin(
        center_points_result.y);
      std::cout << "识别坐标: " << circle_center_x << "," << circle_center_y << std::endl;
      double normal_theta = std::atan2(circle_center_y, circle_center_x);
      if (normal_theta > M_PI) {
        normal_theta = 2 * M_PI - normal_theta;
      }
      Vector2d percept_p(circle_center_x, circle_center_y);
      Vector2d real_p = circle_real(cluster);
      double t_w = calculateTranslationWeight(percept_p, real_p);
      double r_w = calculateRotationWeight(percept_p, real_p);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position.x = circle_center_x;
      pose.pose.position.y = circle_center_y;
      pose.pose.orientation.z = std::sin(normal_theta / 2);
      pose.pose.orientation.w = std::cos(normal_theta / 2);

      CircleFitResult circle = circle_fit(cluster);
      double diameter = 2 * circle.radius;
      double arc_feature = calculate_arc_feature(cluster);
      // 计算置信度
      double confidence = std::min(1.0, 0.8 + 0.2 * (cluster.size() / 20.0)) * 
                          std::min(1.0, circle.r_squared) * 
                          std::min(1.0, arc_feature / arc_threshold);
      Detection tmp_results;
      tmp_results.pose = pose;
      tmp_results.diameter = diameter;
      tmp_results.confidence = confidence;
      tmp_results.translationW = t_w;
      tmp_results.rotationW = r_w;
      current_detections.push_back(tmp_results);
      std::cout << "完成反光柱聚类检测" << std::endl;
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
  return detected_posts;
}

} // namespace landmark_localization