#include "landmark_localization/reflective_post_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace landmark_localization {

  ReflectivePostDetector::ReflectivePostDetector(int intensity_threshold)
  {
    intensity_threshold_use = intensity_threshold;
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

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (msg->intensities[i] > intensity_thresh &&
            msg->ranges[i] > msg->range_min &&
            msg->ranges[i] < msg->range_max &&
          !std::isnan(msg->ranges[i]))
        {
            
          double angle = msg->angle_min + i * msg->angle_increment;
          if(angle > 1.57 && angle < 4.712){
            Vector2d tmp_p;
            tmp_p.x = msg->ranges[i] * std::cos(angle);
            tmp_p.y = msg->ranges[i] * std::sin(angle);
            points.push_back(tmp_p);
            high_intensities.push_back(msg->intensities[i]);
          }
          
        }
    }

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
    unique_labels.erase(
      std::unique(unique_labels.begin(), unique_labels.end()),
      unique_labels.end());
    std::vector<Detection> current_detections;
    for (int label : unique_labels) {
      std::vector<Vector2d> cluster;
      for (size_t i = 0; i < valid_labels.size(); ++i) {
        if (valid_labels[i] == label) {
            cluster.push_back(valid_points[i]);
        }
      }
    //RCLCPP_INFO(this->get_logger(), "info dbscan lable: %d", int(cluster.size()));
    CircleFitResult circle = circle_fit(cluster);
    //RCLCPP_INFO(this->get_logger(), "info arc_feature: %f,%f,%f",circle.center.x,circle.center.y,circle.radius*2);
    if (circle.valid && circle.r_squared >= 0.85) {
      double diameter = 2 * circle.radius;
      if (diameter > diameter_min && diameter < diameter_max) {
        double arc_feature = calculate_arc_feature(cluster);
        /*Vector2d real_p = circle_real(cluster);
        double x_ = -real_p.x;
        double y_ = -real_p.y;
        double normal_theta = std::atan2(y_, x_);
        if (normal_theta > M_PI) {
          normal_theta = 2 * M_PI - normal_theta;
        }
                
        double t_w = calculateTranslationWeight(circle.center, real_p);
        double r_w = calculateRotationWeight(circle.center, real_p);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = real_p.x;
        pose.pose.position.y = real_p.y;
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
        current_detections.push_back(tmp_results);*/
        double max_arc = max_arc_feature;
        //RCLCPP_INFO(this->get_logger(), "info arc_feature: %f",arc_feature);
        if (circle.avg_residual < residual_avg_threshold &&
          circle.std_residual < residual_std_threshold &&
          circle.max_residual < residual_max_threshold &&
          arc_feature > arc_threshold && arc_feature < max_arc)
        {
          double x_ = -circle.center.x;
          double y_ = -circle.center.y;
          double normal_theta = std::atan2(y_, x_);
          if (normal_theta > M_PI) {
            normal_theta = 2 * M_PI - normal_theta;
          }
          Vector2d real_p = circle_real(cluster);
          double t_w = calculateTranslationWeight(circle.center, real_p);
          double r_w = calculateRotationWeight(circle.center, real_p);
          geometry_msgs::msg::PoseStamped pose;
          pose.header = msg->header;
          pose.pose.position.x = circle.center.x;
          pose.pose.position.y = circle.center.y;
          pose.pose.orientation.z = std::sin(normal_theta / 2);
          pose.pose.orientation.w = std::cos(normal_theta / 2);

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
        }
      }
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
        post.translation_weight = detection.translationW * landmark_translation_weight;
        post.rotation_weight = landmark_rotation_weight;
        detected_posts.push_back(post);

        std::cout << "反光柱检测结果：圆心(" << post.position.x << "m, " << post.position.y << "m), t_w=" 
                  << post.translation_weight << ", r_w=" << post.rotation_weight << std::endl;
    }
    return detected_posts;
  }

} // namespace landmark_localization