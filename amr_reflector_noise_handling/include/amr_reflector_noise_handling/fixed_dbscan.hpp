#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include "types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief Fixed-parameter DBSCAN clustering algorithm
 * 
 * Uses fixed EPS=10cm and min_points=5 as specified.
 * Simplified version without multi-scale adaptation.
 */
class FixedDBSCAN {
public:
    /**
     * @brief Constructor with fixed parameters
     */
    FixedDBSCAN()
        : eps_(0.05)      // 10cm clustering radius
        , min_points_(5)    // Minimum 5 points for a cluster
        , visited_(nullptr)
        , cluster_id_(nullptr)
    {}
    
    /**
     * @brief Cluster points using DBSCAN algorithm
     * @param points Input points to cluster
     * @return Vector of clusters, each cluster is a vector of point indices
     */
    std::vector<std::vector<int>> cluster(const std::vector<Point>& points) {
        if (points.empty()) return {};
        
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
        for (auto& cluster : clusters) {
            std::sort(cluster.begin(), cluster.end(), [&](int a, int b) {
                return points[a].origin_index < points[b].origin_index;
            });
        }
        
        // Sort clusters by size (largest first)
        std::sort(clusters.begin(), clusters.end(),
            [](const std::vector<int>& a, const std::vector<int>& b) {
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
    
private:
    double eps_;              // Clustering radius (m)
    int min_points_;          // Minimum points for core point
    bool* visited_;           // Visited flag array
    int* cluster_id_;        // Cluster assignment array
    
    /**
     * @brief Find neighbors within EPS distance
     */
    std::vector<int> findNeighbors(const std::vector<Point>& points, int idx) const {
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
    void expandCluster(const std::vector<Point>& points,
                    int seed_idx,
                    const std::vector<int>& seed_neighbors,
                    int cluster_id) {
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