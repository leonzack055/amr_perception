#include "landmark_localization/reflective_post_detector.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace landmark_localization
{

ReflectivePostDetector::ReflectivePostDetector(int intensity_threshold,int max_age_launch_)
{
  intensity_threshold_use = intensity_threshold;
  max_age_param_ = max_age_launch_;
}

ReflectivePostDetector::~ReflectivePostDetector()
{
}

std::vector<TrackPoint> ReflectivePostDetector::trackWithMOT(const std::vector<TrackPoint>& detections_){
    TrajectorySmoother smoother;
    X5MOTTracker tracker;
    tracker = X5MOTTracker(max_age_param_);
    float timestamp_ = std::chrono::duration<float>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    auto tracked_objects = tracker.processFrame(detections_,timestamp_);
    for(auto& obj_ : tracked_objects){
        smoother.smoothTrajectory(obj_);
    }
    return tracked_objects;
}

std::vector<ReflectivePost> ReflectivePostDetector::detect(const LaserScan & scan)
{
  std::vector<ReflectivePost> detected_posts;

  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int intensity_thresh = intensity_threshold_use;
  // 提取高强度点
  std::vector<Point> points;
  std::vector<float> high_intensities;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (scan.intensities[i] > intensity_thresh &&
      scan.ranges[i] > scan.range_min &&
      scan.ranges[i] < 10 &&
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
  std::vector<TrackPoint> current_detections;
  for (int label : unique_labels) {
    std::vector<Point> cluster;
    for (size_t i = 0; i < valid_labels.size(); ++i) {
      if (valid_labels[i] == label) {
        cluster.push_back(valid_points[i]);
      }
    }
    // 进行高斯拟合
    ReflectorCenterDetector detector(residual_real);
    Point detected_center,orion_p;
    orion_p.x = 0.0;
    orion_p.y = 0.0;
    double confidence_ = 0.0;
    auto cur_time = std::chrono::steady_clock::now();
    auto microseconds = std::chrono::duration<float>(cur_time.time_since_epoch());
    bool result_ = detector.detectReflectorCenter(cluster, detected_center,orion_p,confidence_);
    float result_distance = detected_center.x*detected_center.x + detected_center.y*detected_center.y;
    float result_angle = std::atan2(fabs(detected_center.x),fabs(detected_center.y));
    if(result_angle < 0.52 || fabs(detected_center.x)>4.5){
      result_ = false;
    }
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
      double confidence = confidence_;
      double diameter = residual_real*2;
      current_detections.emplace_back(reflector_center_.x,reflector_center_.y,microseconds.count(),t_w,r_w,confidence,diameter,-1);
      //std::cout << "完成反光柱聚类检测" << std::endl;
    }
  }

  //auto mot_detections = trackWithMOT(current_detections);
  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "检测耗时: " << duration.count() << " 毫秒" << std::endl;

  for (const auto & detection : current_detections) {
    ReflectivePost post;
    post.position.x = detection.x;
    post.position.y = detection.y;
    post.position.z = 0.0;
    post.translation_weight = detection.t_w * landmark_translation_weight;
    post.rotation_weight = landmark_rotation_weight;
    detected_posts.push_back(post);

    std::cout << "反光柱检测结果：圆心(" << post.position.x << "m, " << post.position.y << "m), t_w="
              << post.translation_weight << ", r_w=" << post.rotation_weight << std::endl;
  }
  return detected_posts;
}


std::vector<Detection> ReflectivePostDetector::detect_circles(const LaserScan & scan){
  std::vector<Detection> detected_posts;
  std::vector<TrackPoint> current_detections;
  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  int intensity_thresh = intensity_threshold_use;
  // 提取高强度点
  std::vector<Point> points;
  std::vector<float> high_intensities;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (scan.intensities[i] > intensity_thresh &&
      scan.ranges[i] > scan.range_min &&
      scan.ranges[i] < 10 &&
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
    ReflectorCenterDetector detector(residual_real);
    Point detected_center,orion_p;
    orion_p.x = 0.0;
    orion_p.y = 0.0;
    double confidence_ = 0.0;
    auto cur_time = std::chrono::steady_clock::now();
    auto microseconds = std::chrono::duration<float>(cur_time.time_since_epoch());
    
    float maxDist = distanceMax(cluster);  // 新增：计算聚类内最大距离
    bool result_ = false;
    if(maxDist < 0.004 && maxDist > 0.002){  // 新增：距离范围过滤
        result_ = detector.detectReflectorCenter(cluster, detected_center, orion_p, confidence_);
        float result_angle = std::atan2(fabs(detected_center.x), fabs(detected_center.y));
        if(result_angle < 0.52 || fabs(detected_center.x) > 4.5){
            result_ = false;
        }
    }

    // 2026-01-08 注释，增加距离范围过滤
    // float result_distance = detected_center.x*detected_center.x + detected_center.y*detected_center.y;
    // float result_angle = std::atan2(fabs(detected_center.x),fabs(detected_center.y));
    // if(result_angle < 0.52 || fabs(detected_center.x)>4.5){
    //   result_ = false;
    // }

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
      double confidence = confidence_;
      double diameter = residual_real*2;
      current_detections.emplace_back(reflector_center_.x,reflector_center_.y,microseconds.count(),t_w,r_w,confidence,diameter,-1);
      //std::cout << "完成反光柱聚类检测" << std::endl;
    }
  }

  //auto mot_detections = trackWithMOT(current_detections);
  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "检测耗时: " << duration.count() << " 毫秒" << std::endl;
  for (const auto & detection : current_detections) {
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
    result_.diameter = detection.diameter;
    result_.confidence = detection.confidence;
    result_.translationW = detection.t_w;
    result_.rotationW = detection.r_w;
    detected_posts.push_back(result_);
  }
  return detected_posts;
}
} // namespace landmark_localization
