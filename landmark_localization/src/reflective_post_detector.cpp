#include "landmark_localization/reflective_post_detector.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace landmark_localization
{

ReflectivePostDetector::ReflectivePostDetector(int intensity_threshold)
{
  intensity_threshold_use = intensity_threshold;
}

ReflectivePostDetector::~ReflectivePostDetector()
{
}

std::vector<ReflectivePost> ReflectivePostDetector::detect(const LaserScan & scan)
{
  std::vector<ReflectivePost> detected_posts;

  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int intensity_thresh = intensity_threshold_use;

  auto cur_time = std::chrono::system_clock::now();
  auto microseconds_ = std::chrono::duration_cast<std::chrono::microseconds>(cur_time.time_since_epoch());
  // 提取高强度点
  std::vector<Point> points;
  std::vector<float> high_intensities;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (scan.intensities[i] > intensity_thresh &&
      scan.ranges[i] > scan.range_min &&
      scan.ranges[i] < scan.range_max &&
      !std::isnan(scan.ranges[i]))
    {

      double angle = scan.angle_min + i * scan.angle_increment;
      if (angle > 1.57 && angle < 4.712) {
        Point tmp_p;
        tmp_p.x = scan.ranges[i] * std::cos(angle);
        tmp_p.y = scan.ranges[i] * std::sin(angle);
        tmp_p.intensity = scan.intensities[i];
        points.push_back(tmp_p);
        high_intensities.push_back(scan.intensities[i]);
      }

    }
  }

  if (points.empty()) {
    return detected_posts;
  }

  // 去噪
  if (points.size() > min_cluster_points) {
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
  std::vector<Point> valid_points;
  std::vector<int> valid_labels;
  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] != -1) {
      valid_points.push_back(points[i]);
      valid_labels.push_back(labels[i]);
    }
  }

  if (valid_points.empty()) {
    return detected_posts;
  }
  std::vector<int> unique_labels = valid_labels;
  std::sort(unique_labels.begin(), unique_labels.end());
  unique_labels.erase(
    std::unique(unique_labels.begin(), unique_labels.end()),
    unique_labels.end());
  std::vector<CircleCenter> current_detections;
  for (int label : unique_labels) {
    std::vector<Point> cluster;
    for (size_t i = 0; i < valid_labels.size(); ++i) {
      if (valid_labels[i] == label) {
        cluster.push_back(valid_points[i]);
      }
    }
    // 进行高斯拟合
    ReflectorCenterDetector detector(residual_real, intensity_thresh);
    Point detected_center,orion_p;
    orion_p.x = 0.0;
    orion_p.y = 0.0;
    bool result_ = detector.detectReflectorCenter(cluster, detected_center,orion_p);
    if(result_){
      Point reflector_center_;
      reflector_center_.x = detected_center.x;
      reflector_center_.y = detected_center.y;
      WeightParams params;
      params.ideal_point_count = min_cluster_points;
      params.error_threshold = paramError;
      params.distance_decay = distance_decay;
      
      WeightedCircle weighted_circle = assignWeightsToCircle(cluster, reflector_center_, 0.0005, orion_p, params);
      double t_w = weighted_circle.weight_translation;
      double r_w = weighted_circle.weight_rotation;
      double confidence = 0.99;
      double diameter = residual_real*2;
      CircleCenter tmp_results(reflector_center_.x,reflector_center_.y,microseconds_.count(),t_w,r_w,confidence,diameter,label);
      current_detections.push_back(tmp_results);
      std::cout << "完成反光柱聚类检测" << std::endl;
    }
  }

  // 第一步：相对坐标平滑
  std::vector<CircleCenter> smoothed = smoother.processFrame(current_detections);
  
  // 第二步：几何关系稳定
  std::vector<CircleCenter> stabilized = geometric_stabilizer.stabilizeByGeometry(smoothed);
  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "检测耗时: " << duration.count() << " 毫秒" << std::endl;

  for (const auto & detection : stabilized) {
    ReflectivePost post;
    post.position.x = detection.x;
    post.position.y = detection.y;
    post.position.z = 0.0;
    post.translation_weight = detection.t * landmark_translation_weight;
    post.rotation_weight = landmark_rotation_weight;
    detected_posts.push_back(post);

    std::cout << "反光柱检测结果：圆心(" << post.position.x << "m, " << post.position.y << "m), t_w="
              << post.translation_weight << ", r_w=" << post.rotation_weight << std::endl;
  }
  return detected_posts;
}


std::vector<Detection> ReflectivePostDetector::detect_circles(const LaserScan & scan){
  std::vector<Detection> detected_posts;
  std::vector<CircleCenter> current_detections;
  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int intensity_thresh = intensity_threshold_use;

  auto cur_time = std::chrono::system_clock::now();
  auto microseconds_ = std::chrono::duration_cast<std::chrono::microseconds>(cur_time.time_since_epoch());
  // 提取高强度点
  std::vector<Point> points;
  std::vector<float> high_intensities;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (scan.intensities[i] > intensity_thresh &&
      scan.ranges[i] > scan.range_min &&
      scan.ranges[i] < scan.range_max &&
      !std::isnan(scan.ranges[i]))
    {

      double angle = scan.angle_min + i * scan.angle_increment;
      if (angle > 1.57 && angle < 4.712) {
        Point tmp_p;
        tmp_p.x = scan.ranges[i] * std::cos(angle);
        tmp_p.y = scan.ranges[i] * std::sin(angle);
        tmp_p.intensity = scan.intensities[i];
        points.push_back(tmp_p);
        high_intensities.push_back(scan.intensities[i]);
      }
    }
  }

  if (points.empty()) {
    return detected_posts;
  }

  // 去噪
  if (points.size() > min_cluster_points) {
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
  std::vector<Point> valid_points;
  std::vector<int> valid_labels;
  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] != -1) {
      valid_points.push_back(points[i]);
      valid_labels.push_back(labels[i]);
    }
  }

  if (valid_points.empty()) {
    return detected_posts;
  }
  std::vector<int> unique_labels = valid_labels;
  std::sort(unique_labels.begin(), unique_labels.end());
  unique_labels.erase(
    std::unique(unique_labels.begin(), unique_labels.end()),
    unique_labels.end());
  for (int label : unique_labels) {
    std::vector<Point> cluster;
    for (size_t i = 0; i < valid_labels.size(); ++i) {
      if (valid_labels[i] == label) {
        cluster.push_back(valid_points[i]);
      }
    }
    // 进行高斯拟合
    ReflectorCenterDetector detector(residual_real, intensity_thresh);
    Point detected_center,orion_p;
    orion_p.x = 0.0;
    orion_p.y = 0.0;
    bool result_ = detector.detectReflectorCenter(cluster, detected_center,orion_p);
    if(result_){
      Point reflector_center_;
      reflector_center_.x = detected_center.x;
      reflector_center_.y = detected_center.y;
      WeightParams params;
      params.ideal_point_count = min_cluster_points;
      params.error_threshold = paramError;
      params.distance_decay = distance_decay;
      
      WeightedCircle weighted_circle = assignWeightsToCircle(cluster, reflector_center_, 0.0005, orion_p, params);
      double t_w = weighted_circle.weight_translation;
      double r_w = weighted_circle.weight_rotation;
      double confidence = 0.99;
      double diameter = residual_real*2;
      CircleCenter tmp_results(reflector_center_.x,reflector_center_.y,microseconds_.count(),t_w,r_w,confidence,diameter,label);
      current_detections.push_back(tmp_results);
      std::cout << "完成反光柱聚类检测" << std::endl;
    }
  }

  // 第一步：相对坐标平滑
  std::vector<CircleCenter> smoothed = smoother.processFrame(current_detections);
  
  // 第二步：几何关系稳定
  std::vector<CircleCenter> stabilized = geometric_stabilizer.stabilizeByGeometry(smoothed);
  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "检测耗时: " << duration.count() << " 毫秒" << std::endl;
  for (const auto & detection : stabilized) {
    double x_ = -detection.x;
    double y_ = -detection.y;
    double normal_theta = std::atan2(y_, x_);
    if (normal_theta > M_PI) {
      normal_theta = 2 * M_PI - normal_theta;
    }
    PoseStamped pose;
    pose.header = scan.header;
    pose.pose = transforms::Rigid3d(
      {detection.x, detection.y, 0.0},
      transforms::Rigid3d::AngleAxis(normal_theta, Eigen::Matrix<double, 3, 1>::UnitZ()));
    Detection result_;
    result_.pose = pose;
    result_.diameter = detection.d;
    result_.confidence = detection.c;
    result_.translationW = detection.t;
    result_.rotationW = detection.r;
    detected_posts.push_back(result_);
  }
  return detected_posts;
}
} // namespace landmark_localization
