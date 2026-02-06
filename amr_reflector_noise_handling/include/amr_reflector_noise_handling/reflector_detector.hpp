#ifndef AMR_REFLECTOR_NOISE_HANDLING_REFLECTOR_DETECTOR_HPP
#define AMR_REFLECTOR_NOISE_HANDLING_REFLECTOR_DETECTOR_HPP
#include "amr_reflector_noise_handling/circle_fitter.hpp"
#include "amr_reflector_noise_handling/global_reflector_tracker.hpp"
#include "amr_reflector_noise_handling/pca_shape_classifier.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"
namespace amr_reflector_noise_handling {
class RefelctorDetector {
public:
  void
  configGlobalReflectorTracker(const GlobalReflectorTracker::Config &config);
  void configPCAShapeClassifier(const ShapeClassificationParams &config);
  void configCircleFitter(const CircleFitParams &config);
  std::vector<DetectedReflector>
  detectReflectors(const std::vector<std::vector<Point>> &clusters);
  // TODO: 先检测，再跟踪并利用，优化结果进行全局更新匹配；
  std::vector<DetectedReflector> detectReflectorsWithShortTracking(
      const std::vector<std::vector<Point>> &clusters,
      const transforms::Rigid3d &laser_global_pose, int64_t timestamp);
  std::vector<TrackedReflector> getCurrentAllTrackedReflectors() {
    return std::move(global_reflector_tracker_.getAllTrackedReflectors());
  }
  std::vector<TrackedReflector> getCurrentAllConfirmedTrackedReflectors() {
    return std::move(global_reflector_tracker_.getConfirmedReflectors());
  }
  void setDetectMethod(const std::string& method) {classification_method_ = method;}
private:
  Point computeCentroid(const std::vector<Point> &cluster) {
    Point centroid;
    if (cluster.empty()) {
      return centroid;
    }
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &point : cluster) {
      sum_x += point.x;
      sum_y += point.y;
    }
    centroid.x = sum_x / cluster.size();
    centroid.y = sum_y / cluster.size();
    return centroid;
  }
  std::vector<DetectedReflector> assignedTrackedRefelctors(
      const std::vector<DetectedReflector> &reflectors,
      const std::vector<TrackedReflector> &confirmed_local_reflectors);
  void updateGlobalReflectorTracker(
      const std::vector<DetectedReflector> &detect_reflectors,
      const transforms::Rigid3d &laser_global_pose, int64_t timestamp);
  // 功能器
  GlobalReflectorTracker global_reflector_tracker_;
  PCAShapeClassifier pca_classifier_;
  CircleFitter circle_fitter_;
  // 参数
  double far_distance_threshold_;     // 远近点阈值
  std::string classification_method_; // 使用的方法
};
} // namespace amr_reflector_noise_handling

#endif // !AMR_REFLECTOR_NOISE_HANDLING_REFLECTOR_DETECTOR_HPP
