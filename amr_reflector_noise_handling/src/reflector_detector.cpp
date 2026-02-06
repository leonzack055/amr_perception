#include "amr_reflector_noise_handling/reflector_detector.hpp"
namespace amr_reflector_noise_handling {
void RefelctorDetector::configGlobalReflectorTracker(
    const GlobalReflectorTracker::Config &config) {
  global_reflector_tracker_.setConfig(config);
}

void RefelctorDetector::configPCAShapeClassifier(
    const ShapeClassificationParams &config) {
  pca_classifier_.setParams(config);
  fprintf(stderr,
          "✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪配置PCA检测器参数:✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪\n"
          "[✿]min_points: %d\t[✿]max_elongation_post:%.3f\t"
          "[✿]min_elongation_board:%.3f\n[✿]max_linearity_post:%.3f\t"
          "[✿]min_linearity_board:%.3f\n[✿]near_distance:%.3f\t"
          "[✿]near_min_points:%d\t[✿]near_max_linearity_post:%.3f\n",
          config.min_points, config.max_elongation_post,
          config.min_elongation_board, config.max_linearity_post,
          config.min_linearity_board, config.near_distance,
          config.near_min_points, config.near_max_linearity_post);
}
void RefelctorDetector::configCircleFitter(const CircleFitParams &config) {
  circle_fitter_.setParams(config);
  fprintf(stderr,
          "✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪配置圆拟合参数:✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪✪\n"
          "[✿]max_fit_error:%.3f\t[✿]max_fit_error_far:%.3f\t"
          "[✿]max_fit_error_near:%.3f\n[✿]min_inlier_ratio:%.3f\t"
          "[✿]far_distance_threshold:%.3f\t[✿]min_radius:%.3f\t"
          "[✿]max_radius:%.3f\t[✿]max_concave_ratio:%.3f\n"
          "[✿]ransac_iterations:%d\t[✿]ransac_inlier_threshold:%.3f\t"
          "[✿]ransac_min_points:%d\n",
          config.max_fit_error, config.max_fit_error_far,
          config.max_fit_error_near, config.min_inlier_ratio,
          config.far_distance_threshold, config.min_radius, config.max_radius,
          config.max_concave_ratio, config.ransac_iterations,
          config.ransac_inlier_threshold, config.ransac_min_points);
}

std::vector<DetectedReflector> RefelctorDetector::detectReflectors(
    const std::vector<std::vector<Point>> &clusters) {
  std::vector<DetectedReflector> reflectors;
  reflectors.reserve(clusters.size());
  for (size_t idx = 0; idx < clusters.size(); ++idx) {
    const auto &cluster = clusters[idx];
    Point center = computeCentroid(cluster); // 计算聚类中心
    double distance = center.distanceFromOrigin();
    // Classification
    bool is_reflector_candidate = false;
    if (classification_method_ == "pca") {
      auto pca_features = pca_classifier_.computeShapeFeatures(cluster);
      // 分别根据pca信息和拟合圆信息判别是直线，还是圆弧，以及噪声
      // 噪声检测基本失败
      auto object_type = pca_classifier_.classifyObject(pca_features);
      is_reflector_candidate = (object_type == REFLECTOR_POST);
      fprintf(stderr,
              "PCAClassifier 簇 %zu: 延伸度=%.6g, 线性度=%.3f, 圆形度=%.3f, "
              "聚类点数=%zu, 中心距离=%.3f, 类型=%s\n",
              idx, pca_features.elongation, pca_features.linearity,
              pca_features.circularity, cluster.size(), distance,
              is_reflector_candidate ? "反光柱" : "其他");
    }

    if (!is_reflector_candidate) {
      continue;
    }
    // [TODO:]Interpolation
    // 插值补偿距离较远的稀疏点云，进行弧长插补(Depatched)
    auto processed_cluster = cluster;
    // 圆拟合基本失败,修正圆拟合方法
    auto circle_fit = circle_fitter_.fitArcWithRANSAC(processed_cluster);
    if (!circle_fit.is_valid) {
      fprintf(stderr, "簇 %zu: RANSAC圆拟合失败", idx);
      continue;
    }
    fprintf(stderr,
            "circle_fit 簇 %zu : 中心(%.3f,%.3f), 直径=%.3fm, 总误差=%.6f, "
            "内点数=%d, 内点比例=%.3f, 总点数=%d\n"
            "内点误差=%6f, 外点误差=%6f,  凹半圆检测比率=%4f, "
            "凸半圆检测比率=%4f \n",
            idx, circle_fit.center.x, circle_fit.center.y,
            circle_fit.radius * 2.0, circle_fit.fit_error,
            circle_fit.inlier_count, circle_fit.inlier_ratio,
            circle_fit.total_points, circle_fit.inner_error,
            circle_fit.outline_error, circle_fit.concave_ratio,
            circle_fit.convex_ratio);
    if (circle_fitter_.validateFit(circle_fit, cluster.size(), distance)) {
      DetectedReflector reflector;
      reflector.center = circle_fit.center;
      reflector.diameter = 2 * circle_fit.radius;
      reflector.confidence = 0.8;
      reflector.point_count = cluster.size();
      reflector.idx = idx;
      reflectors.push_back(reflector);
    } else {
      fprintf(stderr, "circle_fit 簇 %zu: RANSAC圆拟合圆拟合 判定失效\n", idx);
    }
    // Compute confidence
  }
  return reflectors;
}

std::vector<DetectedReflector>
RefelctorDetector::detectReflectorsWithShortTracking(
    const std::vector<std::vector<Point>> &clusters,
    const transforms::Rigid3d &laser_global_pose, int64_t timestamp) {
  std::vector<DetectedReflector> detect_reflectors = detectReflectors(clusters);
  updateGlobalReflectorTracker(detect_reflectors, laser_global_pose, timestamp);
  // 1、短时跟踪反光柱过滤
  // 这里获取的是短时内全部的已经确定为真实的激活反光柱；对于想看到
  // 经过跟踪过滤后反光柱位置和判定情况的情况下，需要再重新获取。
  std::vector<TrackedReflector> confirmed_reflectors =
      global_reflector_tracker_.getConfirmedReflectors();
  for (auto &reflector : confirmed_reflectors) {
    auto local_postion = laser_global_pose.inverse() *
                         Eigen::Vector3d(reflector.global_position.x,
                                         reflector.global_position.y, 0.0);
    reflector.global_position.x = local_postion.x();
    reflector.global_position.y = local_postion.y();
  }
  auto real_reflectors =
      assignedTrackedRefelctors(detect_reflectors, confirmed_reflectors);
  return real_reflectors;
}

void RefelctorDetector::updateGlobalReflectorTracker(
    const std::vector<DetectedReflector> &detect_reflectors,
    const transforms::Rigid3d &laser_global_pose, int64_t timestamp) {
  // 全局位姿跟踪更新
  global_reflector_tracker_.update(detect_reflectors, laser_global_pose,
                                   timestamp);
}

/**
 * @brief
 * 计算检测到的Reflectors与跟踪器输出的Confirm之间的结果，来给出确认的检测值
 */
std::vector<DetectedReflector> RefelctorDetector::assignedTrackedRefelctors(
    const std::vector<DetectedReflector> &reflectors,
    const std::vector<TrackedReflector> &confirmed_local_reflectors) {
  std::vector<DetectedReflector> confirmed_reflectors;
  // 计算匹配距离
  // 直接利用确认后的反光柱位置进行匹配
  for (int i = 0; i < reflectors.size(); i++) {
    for (int j = 0; j < confirmed_local_reflectors.size(); j++) {
      if (reflectors[i].center.distanceTo(
              confirmed_local_reflectors[j].global_position) <
          (confirmed_local_reflectors[j].position_std_dev * 2)) {
        confirmed_reflectors.push_back(reflectors[i]);
      }
    }
  }
  return confirmed_reflectors;
}

} // namespace amr_reflector_noise_handling