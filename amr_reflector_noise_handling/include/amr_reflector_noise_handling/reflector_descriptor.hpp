#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "types.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief Scale-invariant descriptor for reflective post
 * 
 * Contains geometric and intensity features normalized to be
 * scale-invariant for matching and tracking.
 */
struct ReflectorDescriptor {
    // Geometric features (scale-normalized)
    double normalized_radius;        // Radius normalized by expected diameter
    double circularity;              // How circular the cluster is [0,1]
    double point_density;            // Points per unit area
    double compactness;              // Compactness measure [0,1]
    
    // Intensity features
    double mean_intensity;            // Mean intensity of cluster
    double intensity_std;            // Standard deviation of intensity
    double intensity_range;           // Max - min intensity
    
    // Radial intensity distribution (scale-invariant)
    std::vector<double> radial_profile;  // Intensity at normalized radial distances
    int radial_bins;                       // Number of radial bins
    
    // Meta information
    int point_count;
    double timestamp;
    
    ReflectorDescriptor()
        : normalized_radius(0.0)
        , circularity(0.0)
        , point_density(0.0)
        , compactness(0.0)
        , mean_intensity(0.0)
        , intensity_std(0.0)
        , intensity_range(0.0)
        , radial_bins(8)
        , point_count(0)
        , timestamp(0.0)
    {
        radial_profile.resize(radial_bins, 0.0);
    }
    
    /**
     * @brief Compute similarity with another descriptor
     * @param other Other descriptor to compare
     * @return Similarity score [0,1], higher means more similar
     */
    double similarity(const ReflectorDescriptor& other) const {
        double sim = 0.0;
        int features = 0;
        
        // Geometric similarity (40% weight)
        if (normalized_radius > 0 && other.normalized_radius > 0) {
            double radius_diff = std::abs(normalized_radius - other.normalized_radius);
            sim += 0.4 * std::exp(-radius_diff * 10.0);
            features++;
        }
        
        if (circularity > 0 && other.circularity > 0) {
            double circ_diff = std::abs(circularity - other.circularity);
            sim += 0.15 * std::exp(-circ_diff * 5.0);
            features++;
        }
        
        if (compactness > 0 && other.compactness > 0) {
            double comp_diff = std::abs(compactness - other.compactness);
            sim += 0.15 * std::exp(-comp_diff * 5.0);
            features++;
        }
        
        // Intensity similarity (30% weight)
        if (mean_intensity > 0 && other.mean_intensity > 0) {
            double intensity_diff = std::abs(mean_intensity - other.mean_intensity) / 
                                 std::max(mean_intensity, other.mean_intensity);
            sim += 0.2 * std::exp(-intensity_diff * 5.0);
            features++;
        }
        
        if (intensity_std > 0 && other.intensity_std > 0) {
            double std_diff = std::abs(intensity_std - other.intensity_std) / 
                             std::max(intensity_std, other.intensity_std);
            sim += 0.1 * std::exp(-std_diff * 5.0);
            features++;
        }
        
        // Radial profile similarity (30% weight)
        if (!radial_profile.empty() && !other.radial_profile.empty()) {
            double profile_sim = computeRadialSimilarity(other.radial_profile);
            sim += 0.3 * profile_sim;
            features++;
        }
        
        // Normalize by number of features
        if (features > 0) {
            sim = sim / features;
        }
        
        return std::max(0.0, std::min(1.0, sim));
    }
    
    /**
     * @brief Update descriptor with new data (exponential moving average)
     * @param new_desc New descriptor to incorporate
     * @param alpha Weight for new data (0-1)
     */
    void update(const ReflectorDescriptor& new_desc, double alpha = 0.3) {
        // Exponential moving average for continuous features
        normalized_radius = (1.0 - alpha) * normalized_radius + alpha * new_desc.normalized_radius;
        circularity = (1.0 - alpha) * circularity + alpha * new_desc.circularity;
        compactness = (1.0 - alpha) * compactness + alpha * new_desc.compactness;
        mean_intensity = (1.0 - alpha) * mean_intensity + alpha * new_desc.mean_intensity;
        intensity_std = (1.0 - alpha) * intensity_std + alpha * new_desc.intensity_std;
        
        // Update radial profile
        if (radial_profile.size() == new_desc.radial_profile.size()) {
            for (size_t i = 0; i < radial_profile.size(); ++i) {
                radial_profile[i] = (1.0 - alpha) * radial_profile[i] + 
                                   alpha * new_desc.radial_profile[i];
            }
        } else if (!new_desc.radial_profile.empty()) {
            radial_profile = new_desc.radial_profile;
        }
        
        point_count = (1.0 - alpha) * point_count + alpha * new_desc.point_count;
    }

private:
    /**
     * @brief Compute similarity between radial profiles
     */
    double computeRadialSimilarity(const std::vector<double>& other_profile) const {
        if (radial_profile.size() != other_profile.size()) {
            return 0.0;
        }
        
        // Cosine similarity
        double dot = 0.0;
        double norm_a = 0.0;
        double norm_b = 0.0;
        
        for (size_t i = 0; i < radial_profile.size(); ++i) {
            dot += radial_profile[i] * other_profile[i];
            norm_a += radial_profile[i] * radial_profile[i];
            norm_b += other_profile[i] * other_profile[i];
        }
        
        if (norm_a < 1e-6 || norm_b < 1e-6) {
            return 0.0;
        }
        
        return dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
    }
};

/**
 * @brief Descriptor extractor for reflective posts
 * 
 * Computes scale-invariant descriptors from point clusters.
 */
class ReflectorDescriptorExtractor {
public:
    ReflectorDescriptorExtractor()
        : expected_diameter_(0.07)  // 7cm default
        , radial_bins_(8)
    {}
    
    /**
     * @brief Extract descriptor from point cluster
     * @param points Cluster points
     * @param center Cluster center
     * @param timestamp Detection timestamp
     * @return Scale-invariant descriptor
     */
    ReflectorDescriptor extract(const std::vector<Point>& points,
                           const Point& center,
                           double timestamp = 0.0) {
        ReflectorDescriptor desc;
        desc.timestamp = timestamp;
        desc.point_count = points.size();
        desc.radial_bins = radial_bins_;
        desc.radial_profile.resize(radial_bins_, 0.0);
        
        if (points.empty()) return desc;
        
        // Compute geometric features
        computeGeometricFeatures(points, center, desc);
        
        // Compute intensity features
        computeIntensityFeatures(points, desc);
        
        // Compute radial intensity profile
        computeRadialProfile(points, center, desc);
        
        return desc;
    }
    
    /**
     * @brief Set expected diameter for normalization
     */
    void setExpectedDiameter(double diameter) {
        expected_diameter_ = diameter;
    }
    
    void setRadialBins(int bins) {
        radial_bins_ = bins;
    }

private:
    double expected_diameter_;  // Expected diameter for normalization (m)
    int radial_bins_;          // Number of radial bins
    
    /**
     * @brief Compute geometric features
     */
    void computeGeometricFeatures(const std::vector<Point>& points,
                               const Point& center,
                               ReflectorDescriptor& desc) {
        // Compute max distance from center
        double max_dist = 0.0;
        double sum_dist = 0.0;
        
        for (const auto& p : points) {
            double d = p.distanceTo(center);
            max_dist = std::max(max_dist, d);
            sum_dist += d;
        }
        
        // Normalize radius by expected diameter
        desc.normalized_radius = max_dist / (expected_diameter_ / 2.0);
        
        // Compute circularity (variance of distances)
        double mean_dist = sum_dist / points.size();
        double variance = 0.0;
        for (const auto& p : points) {
            double d = p.distanceTo(center) - mean_dist;
            variance += d * d;
        }
        variance /= points.size();
        double std_dev = std::sqrt(variance);
        
        // Circularity: lower std_dev means more circular
        desc.circularity = std::exp(-std_dev * 5.0);
        
        // Compute compactness (area vs perimeter)
        double area = 3.14159 * max_dist * max_dist;
        double perimeter = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            size_t next = (i + 1) % points.size();
            perimeter += points[i].distanceTo(points[next]);
        }
        desc.compactness = (4.0 * 3.14159 * area) / (perimeter * perimeter);
        desc.compactness = std::max(0.0, std::min(1.0, desc.compactness));
        
        // Point density
        desc.point_density = points.size() / area;
    }
    
    /**
     * @brief Compute intensity features
     */
    void computeIntensityFeatures(const std::vector<Point>& points,
                             ReflectorDescriptor& desc) {
        if (points.empty()) return;
        
        // Mean intensity
        double sum = 0.0;
        for (const auto& p : points) {
            sum += p.intensity;
        }
        desc.mean_intensity = sum / points.size();
        
        // Standard deviation
        double variance = 0.0;
        for (const auto& p : points) {
            double diff = p.intensity - desc.mean_intensity;
            variance += diff * diff;
        }
        variance /= points.size();
        desc.intensity_std = std::sqrt(variance);
        
        // Range
        double min_i = points[0].intensity;
        double max_i = points[0].intensity;
        for (const auto& p : points) {
            min_i = std::min(min_i, p.intensity);
            max_i = std::max(max_i, p.intensity);
        }
        desc.intensity_range = max_i - min_i;
    }
    
    /**
     * @brief Compute radial intensity profile
     */
    void computeRadialProfile(const std::vector<Point>& points,
                           const Point& center,
                           ReflectorDescriptor& desc) {
        if (points.empty() || desc.radial_profile.empty()) return;
        
        // Compute max distance for normalization
        double max_dist = 0.0;
        for (const auto& p : points) {
            max_dist = std::max(max_dist, p.distanceTo(center));
        }
        
        // Accumulate intensities in radial bins
        std::vector<int> bin_counts(desc.radial_bins, 0);
        for (const auto& p : points) {
            double dist = p.distanceTo(center);
            int bin = static_cast<int>((dist / max_dist) * desc.radial_bins);
            bin = std::max(0, std::min(desc.radial_bins - 1, bin));
            
            desc.radial_profile[bin] += p.intensity;
            bin_counts[bin]++;
        }
        
        // Normalize each bin by count
        for (int i = 0; i < desc.radial_bins; ++i) {
            if (bin_counts[i] > 0) {
                desc.radial_profile[i] /= bin_counts[i];
            }
        }
        
        // Normalize profile to [0,1]
        double max_val = 0.0;
        for (double val : desc.radial_profile) {
            max_val = std::max(max_val, val);
        }
        if (max_val > 0) {
            for (auto& val : desc.radial_profile) {
                val /= max_val;
            }
        }
    }
};

} // namespace amr_reflector_noise_handling