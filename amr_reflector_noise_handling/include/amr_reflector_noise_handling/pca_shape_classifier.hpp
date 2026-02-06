#ifndef AMR_REFELCTOR_NOISE_HANDLING_PCA_SHAPE_CLASSIFIER_HPP
#define AMR_REFELCTOR_NOISE_HANDLING_PCA_SHAPE_CLASSIFIER_HPP

#include "amr_reflector_noise_handling/types.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace amr_reflector_noise_handling {

/**
 * @brief 形状特征（基于PCA）
 */
struct ShapeFeatures {
  Point center;
  double eigenvalue1; // 第一主成分（最大方差方向）
  double eigenvalue2; // 第二主成分（最小方差方向）
  double elongation;  // 延伸度 = sqrt(eigenvalue1 / eigenvalue2)
  double orientation; // 主方向角度（弧度）
  double linearity;   // 线性度
  double circularity; // 圆形度
  int point_count;

  ShapeFeatures()
      : eigenvalue1(0), eigenvalue2(0), elongation(0), orientation(0),
        linearity(0), circularity(0), point_count(0) {}
};

/**
 * @brief 形状分类参数
 */
struct ShapeClassificationParams {
  double max_elongation_post = 9.5;      // 反光柱最大延伸度
  double min_elongation_board = 3.0;     // 反光板最小延伸度
  double max_linearity_post = 0.7;       // 反光板最小线性度
  double min_linearity_board = 0.5;      // 反光柱最大线性度
  double near_distance = 1.3;            // 近距离二次判定
  int min_points = 13;                    // 远处聚类点最小点数判定
  int near_min_points = 20;              // 二次判定近距离聚类点数
  double near_max_linearity_post = 0.89; // 二次判定线性度
};

/**
 * @brief 物体类型（基于形状特征）
 */
enum ObjectType {
  REFLECTOR_POST,  // 反光柱
  REFLECTOR_BOARD, // 反光板
  CLUSTER_NOISE,   // 团点噪声
  OBJECT_UNKNOWN   // 未知物体
};

/**
 * @brief 形状分类器（基于PCA）
 *
 * 利用PCA提取形状特征，区分反光柱（圆形）、反光板（线状）和噪声
 */
class PCAShapeClassifier {
public:
  PCAShapeClassifier() = default;

  /**
   * @brief 设置分类参数
   */
  void setParams(const ShapeClassificationParams &params) { params_ = params; }

  /**
   * @brief 计算形状特征（基于PCA）
   *
   * @param points 点云
   * @return ShapeFeatures 形状特征
   */
  ShapeFeatures computeShapeFeatures(const std::vector<Point> &points) const {

    ShapeFeatures features;
    features.point_count = points.size();

    if (points.empty()) {
      return features;
    }

    // 1. 计算中心
    features.center = computeCentroid(points);

    // 2. 计算协方差矩阵
    double cov_xx = 0, cov_yy = 0, cov_xy = 0;
    for (const auto &p : points) {
      double dx = p.x - features.center.x;
      double dy = p.y - features.center.y;
      cov_xx += dx * dx;
      cov_yy += dy * dy;
      cov_xy += dx * dy;
    }
    cov_xx /= points.size();
    cov_yy /= points.size();
    cov_xy /= points.size();

    // 3. 计算特征值和特征向量
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double discriminant = std::sqrt(std::max(0.0, trace * trace - 4 * det));

    features.eigenvalue1 = (trace + discriminant) / 2;
    features.eigenvalue2 = (trace - discriminant) / 2;

    // 4. 计算延伸度
    if (features.eigenvalue2 > 1e-6) {
      features.elongation =
          std::sqrt(features.eigenvalue1 / features.eigenvalue2);
    } else {
      features.elongation = std::numeric_limits<double>::max();
    }

    // 5. 计算主方向
    features.orientation = 0.5 * std::atan2(2 * cov_xy, cov_xx - cov_yy);

    // 6. 计算线性度
    if (features.eigenvalue1 + features.eigenvalue2 > 1e-6) {
      features.linearity = (features.eigenvalue1 - features.eigenvalue2) /
                           (features.eigenvalue1 + features.eigenvalue2);
    } else {
      features.linearity = 0.0;
    }

    features.circularity = 0.0;

    return features;
  }

  /**
   * @brief 基于形状特征分类物体
   *
   * @param features 形状特征
   * @return ObjectType 物体类型
   */
  ObjectType classifyObject(const ShapeFeatures &features) const {

    // 规则4: 点数过少 → 噪声
    if (features.point_count < params_.min_points) {
      return CLUSTER_NOISE;
    }

    // 规则1: 圆形度高 → 反光柱
    if (features.elongation <= params_.max_elongation_post &&
        features.linearity <= params_.max_linearity_post) {
      // 规则5： 二次判定近距离聚类数目要大于20个点且线性度要高 否则为噪声
      if (features.center.distanceFromOrigin() < params_.near_distance) {
        if (features.point_count < params_.near_min_points || features.linearity > params_.near_max_linearity_post) {
          return CLUSTER_NOISE;
        }
      }
      return REFLECTOR_POST;
    }

    // 规则2: 延伸度大 + 线性度高 → 反光板
    if (features.elongation >= params_.min_elongation_board &&
        features.linearity >= params_.min_linearity_board) {
      return REFLECTOR_BOARD;
    }

    return OBJECT_UNKNOWN;
  }

  /**
   * @brief 获取类型名称（用于日志）
   */
  static std::string getTypeName(ObjectType type) {
    switch (type) {
    case REFLECTOR_POST:
      return "反光柱";
    case REFLECTOR_BOARD:
      return "反光板";
    case CLUSTER_NOISE:
      return "噪声";
    case OBJECT_UNKNOWN:
    default:
      return "未知";
    }
  }

  /**
   * @brief 计算角度覆盖率（公共方法）
   */
  double computeAngularCoverage(const std::vector<Point> &points,
                                const Point &center) const {
    if (points.size() < 2) {
      return 0.0;
    }

    // 1. 计算所有角度
    std::vector<double> angles;
    for (const auto &p : points) {
      angles.push_back(std::atan2(p.y - center.y, p.x - center.x));
    }
    std::sort(angles.begin(), angles.end());

    // 2. 计算最大角度间隙
    double max_gap = 0;
    for (size_t i = 0; i < angles.size(); ++i) {
      double gap = angles[(i + 1) % angles.size()] - angles[i];
      if (gap < 0)
        gap += 2 * M_PI;
      max_gap = std::max(max_gap, gap);
    }

    // 3. 计算角度覆盖率
    return 2 * M_PI - max_gap;
  }

private:
  ShapeClassificationParams params_;

  /**
   * @brief 计算中心
   */
  Point computeCentroid(const std::vector<Point> &points) const {
    Point centroid;
    if (points.empty()) {
      centroid.x = 0.0;
      centroid.y = 0.0;
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &point : points) {
      sum_x += point.x;
      sum_y += point.y;
    }

    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    return centroid;
  }
};

} // namespace amr_reflector_noise_handling

#endif // !AMR_REFELCTOR_NOISE_HANDLING_PCA_SHAPE_CLASSIFIER_HPP