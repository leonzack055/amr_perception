#pragma once

#include "reflector_descriptor.hpp"
#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace amr_reflector_noise_handling {

/**
 * @brief Legacy tracked reflector information (descriptor-based tracking)
 *
 * @note This is a legacy implementation used by ReflectorTracker.
 * For global tracking, see GlobalReflectorTracker and TrackedReflector in
 * types.hpp
 */
struct LegacyTrackedReflector {
  int id;                         // Unique tracking ID
  ReflectorDescriptor descriptor; // Current descriptor
  Point position;                 // Estimated position
  rclcpp::Time last_seen;         // Last detection time
  std::vector<Point> point_cloud; // Accumulated point cloud
  int detection_count;            // Number of detections
  double confidence;              // Tracking confidence [0,1]

  LegacyTrackedReflector() : id(-1), detection_count(0), confidence(0.0) {}
};

/**
 * @brief Reflector tracking system with ID matching and point cloud fusion
 *
 * Tracks reflectors over time using scale-invariant descriptors,
 * maintains IDs within a time window (2 seconds), and fuses
 * point clouds from multiple detections.
 */
class ReflectorTracker {
public:
  /**
   * @brief Constructor
   */
  ReflectorTracker()
      : max_time_gap_(2.0) // 2 seconds max time gap
        ,
        match_threshold_(0.8) // 0.8 similarity threshold
        ,
        next_id_(0), node_(nullptr) {}

  /**
   * @brief Update tracking with new detections
   * @param detections New detected reflectors
   * @param timestamp Detection timestamp
   * @return Vector of tracked reflectors with IDs
   */
  std::vector<LegacyTrackedReflector>
  update(const std::vector<DetectedReflector> &detections,
         const rclcpp::Time &timestamp) {

    std::vector<LegacyTrackedReflector> result;

    // Step 1: Match new detections to existing tracks
    std::vector<bool> matched(detections.size(), false);

    for (size_t i = 0; i < detections.size(); ++i) {
      auto &detection = detections[i];

      // Extract descriptor from detection
      auto desc = extractDescriptor(detection, timestamp.seconds());

      // Find best match
      int best_match_id = -1;
      double best_similarity = 0.0;

      for (auto &track : tracked_reflectors_) {
        double similarity = desc.similarity(track.descriptor);

        if (similarity > best_similarity) {
          best_similarity = similarity;
          best_match_id = track.id;
        }
      }

      // Check if match is good enough
      if (best_similarity >= match_threshold_ && best_match_id >= 0) {
        // Update existing track
        updateTrack(best_match_id, detection, desc, timestamp);
        matched[i] = true;

        // Find the track and add to result
        for (auto &track : tracked_reflectors_) {
          if (track.id == best_match_id) {
            result.push_back(track);
            break;
          }
        }
      } else {
        // Create new track
        int new_id = createNewTrack(detection, desc, timestamp);
        matched[i] = true;

        // Find the new track and add to result
        for (auto &track : tracked_reflectors_) {
          if (track.id == new_id) {
            result.push_back(track);
            break;
          }
        }
      }
    }

    // Step 2: Remove expired tracks
    removeExpiredTracks(timestamp);

    // Step 3: Remove unmatched tracks (tracks that weren't updated)
    removeUnmatchedTracks(timestamp);

    return result;
  }

  /**
   * @brief Get all currently tracked reflectors
   */
  const std::vector<LegacyTrackedReflector> &getTracks() const {
    return tracked_reflectors_;
  }

  /**
   * @brief Set ROS2 node for logging
   */
  void setNode(rclcpp::Node *node) { node_ = node; }

  /**
   * @brief Set tracking parameters
   */
  void setMaxTimeGap(double gap) { max_time_gap_ = gap; }
  void setMatchThreshold(double threshold) { match_threshold_ = threshold; }

  /**
   * @brief Get tracking parameters
   */
  double getMaxTimeGap() const { return max_time_gap_; }
  double getMatchThreshold() const { return match_threshold_; }

private:
  double max_time_gap_;    // Maximum time gap for track continuity (s)
  double match_threshold_; // Similarity threshold for matching
  int next_id_;            // Next available ID
  std::vector<LegacyTrackedReflector> tracked_reflectors_; // Active tracks
  rclcpp::Node *node_; // ROS2 node for logging

  /**
   * @brief Extract descriptor from detection
   */
  ReflectorDescriptor extractDescriptor(const DetectedReflector &detection,
                                        double timestamp) {

    ReflectorDescriptorExtractor extractor;
    extractor.setExpectedDiameter(0.07); // 7cm

    // Note: We don't have the full point cloud here,
    // so we create a simplified descriptor
    ReflectorDescriptor desc;
    desc.mean_intensity = detection.mean_intensity;
    desc.timestamp = timestamp;
    desc.point_count = detection.point_count;
    desc.normalized_radius = (detection.diameter / 2.0) / 0.07;
    desc.circularity = detection.confidence; // Use confidence as proxy
    desc.compactness = detection.confidence;

    return desc;
  }

  /**
   * @brief Update existing track with new detection
   */
  void updateTrack(int track_id, const DetectedReflector &detection,
                   const ReflectorDescriptor &new_desc,
                   const rclcpp::Time &timestamp) {

    for (auto &track : tracked_reflectors_) {
      if (track.id == track_id) {
        // Update position (weighted average)
        double alpha = 0.3;
        track.position.x =
            (1.0 - alpha) * track.position.x + alpha * detection.center.x;
        track.position.y =
            (1.0 - alpha) * track.position.y + alpha * detection.center.y;

        // Update descriptor (exponential moving average)
        track.descriptor.update(new_desc, alpha);

        // Update timestamp
        track.last_seen = timestamp;

        // Increment detection count
        track.detection_count++;

        // Update confidence based on consistency
        track.confidence = std::min(1.0, track.confidence + 0.1);

        // Fuse point cloud
        if (detection.point_count > 0) {
          fusePointCloud(track, detection.center, detection.diameter);
        }

        if (node_) {
          RCLCPP_DEBUG(node_->get_logger(),
                       "Updated track %d (detections: %d, confidence: %.2f)",
                       track.id, track.detection_count, track.confidence);
        }
        break;
      }
    }
  }

  /**
   * @brief Create new track from detection
   */
  int createNewTrack(const DetectedReflector &detection,
                     const ReflectorDescriptor &desc,
                     const rclcpp::Time &timestamp) {

    LegacyTrackedReflector new_track;
    new_track.id = next_id_++;
    new_track.descriptor = desc;
    new_track.position = detection.center;
    new_track.last_seen = timestamp;
    new_track.detection_count = 1;
    new_track.confidence = 0.5; // Initial confidence

    // Initialize point cloud (simplified)
    // In a full implementation, we would have access to the actual points
    new_track.point_cloud.clear();

    tracked_reflectors_.push_back(new_track);

    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), "Created new track %d at (%.3f, %.3f)",
                   new_track.id, new_track.position.x, new_track.position.y);
    }

    return new_track.id;
  }

  /**
   * @brief Remove expired tracks
   */
  void removeExpiredTracks(const rclcpp::Time &current_time) {
    auto it = tracked_reflectors_.begin();

    while (it != tracked_reflectors_.end()) {
      double time_diff = (current_time - it->last_seen).seconds();

      if (time_diff > max_time_gap_) {
        if (node_) {
          RCLCPP_DEBUG(node_->get_logger(),
                       "Removed expired track %d (time gap: %.2f s)", it->id,
                       time_diff);
        }
        it = tracked_reflectors_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /**
   * @brief Remove unmatched tracks (tracks that weren't updated in this cycle)
   */
  void removeUnmatchedTracks(const rclcpp::Time & /*current_time*/) {
    // This is handled by the timestamp update in updateTrack
    // Tracks that weren't updated will eventually be removed by
    // removeExpiredTracks
  }

  /**
   * @brief Fuse point cloud into track
   *
   * In a full implementation, this would accumulate and merge point clouds
   * from multiple detections to maintain a global consistent representation.
   */
  void fusePointCloud(LegacyTrackedReflector &track, const Point &center,
                      double /*diameter*/) {
    // Simplified: just keep the latest point count
    // In a full implementation, we would:
    // 1. Register point clouds using ICP or similar
    // 2. Merge overlapping points
    // 3. Update the global descriptor

    // For now, we just update the point count estimate
    if (track.point_cloud.empty()) {
      track.point_cloud.push_back(center);
    } else {
      // Simple averaging
      track.point_cloud[0].x = (track.point_cloud[0].x + center.x) / 2.0;
      track.point_cloud[0].y = (track.point_cloud[0].y + center.y) / 2.0;
    }
  }
};

} // namespace amr_reflector_noise_handling