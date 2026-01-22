#pragma once

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <memory>

#include "types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief Union-Find data structure for efficient clustering
 */
class UnionFind {
public:
    UnionFind(int n) : parent_(n), rank_(n, 0) {
        for (int i = 0; i < n; ++i) parent_[i] = i;
    }
    
    int find(int x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);
        }
        return parent_[x];
    }
    
    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank_[rootX] < rank_[rootY]) {
                parent_[rootX] = rootY;
            } else if (rank_[rootX] > rank_[rootY]) {
                parent_[rootY] = rootX;
            } else {
                parent_[rootY] = rootX;
                rank_[rootX]++;
            }
        }
    }

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

/**
 * @brief Multi-scale DBSCAN clustering
 * 
 * Implements a two-stage clustering approach:
 * 1. Coarse clustering with large EPS to identify potential regions
 * 2. Fine clustering with small EPS within each coarse cluster
 */
class MultiScaleDBSCAN {
public:
    /**
     * @brief Constructor with default parameters
     */
    MultiScaleDBSCAN()
        : coarse_eps_(0.15)
        , fine_eps_(0.065)
        , min_points_coarse_(3)
        , min_points_fine_(4)
    {}
    
    /**
     * @brief Constructor with custom parameters
     * @param coarse_eps Coarse clustering neighborhood radius
     * @param fine_eps Fine clustering neighborhood radius
     * @param min_points_coarse Minimum points for coarse clustering
     * @param min_points_fine Minimum points for fine clustering
     */
    MultiScaleDBSCAN(double coarse_eps, double fine_eps,
                   int min_points_coarse, int min_points_fine)
        : coarse_eps_(coarse_eps)
        , fine_eps_(fine_eps)
        , min_points_coarse_(min_points_coarse)
        , min_points_fine_(min_points_fine)
    {}
    
    /**
     * @brief Perform multi-scale DBSCAN clustering
     * @param points Points to cluster
     * @return Vector of clusters, each cluster is a vector of point indices
     */
    std::vector<std::vector<int>> cluster(const std::vector<Point>& points) {
        if (points.empty()) return {};
        
        // Stage 1: Coarse clustering
        auto coarse_labels = dbscan(points, coarse_eps_, min_points_coarse_);
        auto coarse_clusters = extractClusters(coarse_labels);
        
        // Stage 2: Fine clustering within each coarse cluster
        std::vector<std::vector<int>> all_clusters;
        
        for (const auto& coarse_cluster : coarse_clusters) {
            if (coarse_cluster.size() < min_points_fine_) continue;
            
            // Extract points in this coarse cluster
            std::vector<Point> cluster_points;
            std::vector<int> index_map;
            for (int idx : coarse_cluster) {
                cluster_points.push_back(points[idx]);
                index_map.push_back(idx);
            }
            
            // Fine clustering
            auto fine_labels = dbscan(cluster_points, fine_eps_, min_points_fine_);
            auto fine_clusters = extractClusters(fine_labels);
            
            // Map back to original indices
            for (const auto& fine_cluster : fine_clusters) {
                if (fine_cluster.size() < min_points_fine_) continue;
                
                std::vector<int> original_indices;
                for (int idx : fine_cluster) {
                    original_indices.push_back(index_map[idx]);
                }
                all_clusters.push_back(original_indices);
            }
        }
        
        return all_clusters;
    }
    
    /**
     * @brief Set coarse EPS
     */
    void setCoarseEps(double eps) {
        coarse_eps_ = eps;
    }
    
    /**
     * @brief Set fine EPS
     */
    void setFineEps(double eps) {
        fine_eps_ = eps;
    }
    
    /**
     * @brief Set minimum points for coarse clustering
     */
    void setMinPointsCoarse(int min_points) {
        min_points_coarse_ = min_points;
    }
    
    /**
     * @brief Set minimum points for fine clustering
     */
    void setMinPointsFine(int min_points) {
        min_points_fine_ = min_points;
    }

private:
    double coarse_eps_;       // Coarse clustering neighborhood radius
    double fine_eps_;         // Fine clustering neighborhood radius
    int min_points_coarse_;   // Minimum points for coarse clustering
    int min_points_fine_;     // Minimum points for fine clustering
    
    /**
     * @brief Standard DBSCAN implementation
     * @param points Points to cluster
     * @param eps Neighborhood radius
     * @param min_points Minimum points to form a cluster
     * @return Cluster labels (-1 for noise)
     */
    std::vector<int> dbscan(const std::vector<Point>& points,
                            double eps,
                            int min_points) {
        const int n = points.size();
        if (n == 0) return {};
        
        std::vector<int> labels(n, -1);
        std::vector<bool> visited(n, false);
        int cluster_id = 0;
        
        // Precompute grid for faster neighbor search
        auto grid = buildGrid(points, eps);
        
        for (int i = 0; i < n; ++i) {
            if (visited[i]) continue;
            
            auto neighbors = findNeighbors(points, i, eps, grid);
            
            if (neighbors.size() < static_cast<size_t>(min_points)) {
                labels[i] = -1; // Noise
                continue;
            }
            
            visited[i] = true;
            labels[i] = cluster_id;
            
            // Expand cluster
            std::vector<int> seeds = neighbors;
            size_t seed_idx = 0;
            
            while (seed_idx < seeds.size()) {
                int current = seeds[seed_idx++];
                
                if (!visited[current]) {
                    visited[current] = true;
                    auto current_neighbors = findNeighbors(points, current, eps, grid);
                    
                    if (current_neighbors.size() >= static_cast<size_t>(min_points)) {
                        for (int neighbor : current_neighbors) {
                            if (labels[neighbor] == -1) {
                                labels[neighbor] = cluster_id;
                                seeds.push_back(neighbor);
                            }
                        }
                    }
                }
            }
            
            cluster_id++;
        }
        
        return labels;
    }
    
    /**
     * @brief Build spatial grid for efficient neighbor search
     */
    std::map<std::pair<int, int>, std::vector<int>> buildGrid(
        const std::vector<Point>& points,
        double eps) const {
        
        std::map<std::pair<int, int>, std::vector<int>> grid;
        double grid_size = eps;
        
        for (size_t i = 0; i < points.size(); ++i) {
            int grid_x = static_cast<int>(points[i].x / grid_size);
            int grid_y = static_cast<int>(points[i].y / grid_size);
            grid[{grid_x, grid_y}].push_back(i);
        }
        
        return grid;
    }
    
    /**
     * @brief Find neighbors using grid acceleration
     */
    std::vector<int> findNeighbors(
        const std::vector<Point>& points,
        int query_idx,
        double eps,
        const std::map<std::pair<int, int>, std::vector<int>>& grid) const {
        
        std::vector<int> neighbors;
        const auto& query_point = points[query_idx];
        
        int grid_x = static_cast<int>(query_point.x / eps);
        int grid_y = static_cast<int>(query_point.y / eps);
        
        // Search 3x3 neighboring grid cells
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                auto it = grid.find({grid_x + dx, grid_y + dy});
                if (it != grid.end()) {
                    for (int idx : it->second) {
                        if (query_point.squaredDistanceTo(points[idx]) < eps * eps) {
                            neighbors.push_back(idx);
                        }
                    }
                }
            }
        }
        
        return neighbors;
    }
    
    /**
     * @brief Extract clusters from labels
     */
    std::vector<std::vector<int>> extractClusters(const std::vector<int>& labels) const {
        std::map<int, std::vector<int>> cluster_map;
        int max_cluster_id = 0;
        
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] != -1) {
                cluster_map[labels[i]].push_back(i);
                max_cluster_id = std::max(max_cluster_id, labels[i]);
            }
        }
        
        std::vector<std::vector<int>> clusters;
        for (int id = 0; id <= max_cluster_id; ++id) {
            if (cluster_map.find(id) != cluster_map.end()) {
                clusters.push_back(cluster_map[id]);
            }
        }
        
        return clusters;
    }
};

} // namespace amr_reflector_noise_handling