#pragma once

#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace amr_reflector_noise_handling {

/**
 * @brief Fixed-parameter DBSCAN clustering algorithm
 *
 * Uses fixed EPS=10cm and min_points=5 as specified.
 * Simplified version without multi-scale adaptation.
 */
class FixedDBSCAN {
public:
  struct Config {
    double eps = 0.05;
    int min_points = 5;
    int gap_threshold = 3;
    int continue_points = 8;
  };
  FixedDBSCAN(const Config &config)
      : eps_(config.eps) // 10cm clustering radius
        ,
        min_points_(config.min_points) // Minimum 5 points for a cluster
        ,
        gap_threshold_(config.gap_threshold),
        continue_points_(config.continue_points), visited_(nullptr),
        cluster_id_(nullptr) {}
  /**
   * @brief Constructor with fixed parameters
   */
  FixedDBSCAN()
      : eps_(0.05) // 10cm clustering radius
        ,
        min_points_(5) // Minimum 5 points for a cluster
        ,
        gap_threshold_(3), continue_points_(8), visited_(nullptr),
        cluster_id_(nullptr) {}
  std::vector<std::vector<Point>> splitCluster(std::vector<Point> &points) {
    auto cluster_idxs = cluster(points);
    auto split_cluster_idxs =
        continueClusterDetector(points, cluster_idxs, gap_threshold_);
    std::vector<std::vector<Point>> clusters =
        splitContinueClusters(points, split_cluster_idxs);
    std::cerr << "DBSCAN聚类得到" << cluster_idxs.size() << "个簇 "
              << std::endl;
    std::cerr << "DBSCAN连续性分割后,得到" << split_cluster_idxs.size()
              << "个簇" << std::endl;
    return clusters;
  }
  /**
   * @brief Cluster points using DBSCAN algorithm
   * @param points Input points to cluster
   * @return Vector of clusters, each cluster is a vector of point indices
   */
  std::vector<std::vector<int>> cluster(const std::vector<Point> &points) {
    if (points.empty())
      return {};

    int n = points.size();
    visited_ = new bool[n];
    cluster_id_ = new int[n];
    std::fill(visited_, visited_ + n, false);
    std::fill(cluster_id_, cluster_id_ + n, -1);

    int current_cluster = 0;

    for (int i = 0; i < n; ++i) {
      if (!visited_[i]) {
        visited_[i] = true;
        auto neighbors = findNeighbors(points, i);

        if (static_cast<int>(neighbors.size()) >= min_points_) {
          // Start a new cluster
          expandCluster(points, i, neighbors, current_cluster);
          current_cluster++;
        }
      }
    }

    // Collect clusters
    std::vector<std::vector<int>> clusters(current_cluster);
    for (int i = 0; i < n; ++i) {
      if (cluster_id_[i] >= 0) {
        clusters[cluster_id_[i]].push_back(i);
      }
    }
    // 以点云升序排序
    for (auto &cluster : clusters) {
      std::sort(cluster.begin(), cluster.end(), [&](int a, int b) {
        return points[a].origin_index < points[b].origin_index;
      });
    }

    // Sort clusters by size (largest first)
    std::sort(clusters.begin(), clusters.end(),
              [](const std::vector<int> &a, const std::vector<int> &b) {
                return a.size() > b.size();
              });

    cleanup();
    return clusters;
  }

  /**
   * @brief Set clustering parameters
   */
  void setEPS(double eps) { eps_ = eps; }
  void setMinPoints(int min_pts) { min_points_ = min_pts; }

  /**
   * @brief
   * 利用聚类的索引序列，判断聚类的连续性，如果中间有断开，则认为是两个聚类
   */
  std::vector<std::vector<int>>
  continueClusterDetector(const std::vector<Point> &filtered_points,
                          const std::vector<std::vector<int>> &cluster_indices,
                          int gap_threshold) {
    std::vector<std::vector<int>> new_cluster_indices;
    int cluster_idx = 0;
    for (const auto &indices : cluster_indices) {
      std::vector<int> new_cluster;
      for (std::vector<int>::const_iterator iter = indices.begin();
           iter < indices.end() - 1; ++iter) {
        int index_gap = filtered_points[*(iter + 1)].origin_index -
                        filtered_points[*iter].origin_index;
        // gap设置为3 超过3个重新打断分类
        if (index_gap > gap_threshold) {
          std::vector<int> new_split_cluster(new_cluster);
          if (new_split_cluster.size() > continue_points_) {
            new_cluster_indices.push_back(new_split_cluster);
          }
          new_cluster.clear();
        }
        new_cluster.push_back(*iter);
      }
      if (new_cluster.size() > continue_points_) {
        new_cluster_indices.push_back(new_cluster);
      }
      cluster_idx++;
    }
    return new_cluster_indices;
  }

  std::vector<std::vector<Point>> splitContinueClusters(
      std::vector<Point> &filtered_points,
      const std::vector<std::vector<int>> &split_cluster_idxs) {
    std::vector<std::vector<Point>> clusters;
    int cluster_idx = 0;
    for (const auto &indices : split_cluster_idxs) {
      std::vector<Point> cluster;
      cluster.reserve(indices.size());
      for (int idx : indices) {
        cluster.push_back(filtered_points[idx]);
        filtered_points[idx].intensity = 2000 + cluster_idx * 200;
      }
      clusters.push_back(cluster);
      cluster_idx++;
    }
    return clusters;
  }

private:
  double eps_;          // Clustering radius (m)
  int min_points_;      // Minimum points for core point
  int gap_threshold_;   // 用于连续性检测
  int continue_points_; // 连续性检测形成的最小聚类簇
  bool *visited_;       // Visited flag array
  int *cluster_id_;     // Cluster assignment array

  /**
   * @brief Find neighbors within EPS distance
   */
  std::vector<int> findNeighbors(const std::vector<Point> &points,
                                 int idx) const {
    std::vector<int> neighbors;
    double eps_sq = eps_ * eps_;

    for (size_t i = 0; i < points.size(); ++i) {
      if (points[idx].squaredDistanceTo(points[i]) <= eps_sq) {
        neighbors.push_back(i);
      }
    }

    return neighbors;
  }

  /**
   * @brief Expand cluster from seed point
   */
  void expandCluster(const std::vector<Point> &points, int seed_idx,
                     const std::vector<int> &seed_neighbors, int cluster_id) {
    cluster_id_[seed_idx] = cluster_id;

    std::vector<int> seeds = seed_neighbors;
    size_t current_idx = 0;

    while (current_idx < seeds.size()) {
      int current = seeds[current_idx];

      if (!visited_[current]) {
        visited_[current] = true;
        auto neighbors = findNeighbors(points, current);

        if (static_cast<int>(neighbors.size()) >= min_points_) {
          // Add all unclustered neighbors to seeds
          for (int neighbor : neighbors) {
            if (cluster_id_[neighbor] < 0) {
              cluster_id_[neighbor] = cluster_id;
              seeds.push_back(neighbor);
            }
          }
        }
      }

      current_idx++;
    }
  }

  /**
   * @brief Clean up allocated memory
   */
  void cleanup() {
    if (visited_) {
      delete[] visited_;
      visited_ = nullptr;
    }
    if (cluster_id_) {
      delete[] cluster_id_;
      cluster_id_ = nullptr;
    }
  }
};

} // namespace amr_reflector_noise_handling