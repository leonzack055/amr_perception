#include "amr_reflector_noise_handling/global_reflector_tracker.hpp"

#include <rclcpp/rclcpp.hpp>

namespace amr_reflector_noise_handling {

GlobalReflectorTracker::GlobalReflectorTracker(const Config &config)
    : config_(config), next_global_id_(0), current_timestamp_(0) {
  RCLCPP_INFO(rclcpp::get_logger("GlobalReflectorTracker"),
              "Global Reflector Tracker initialized");
}

GlobalReflectorTracker::~GlobalReflectorTracker() {
  RCLCPP_INFO(rclcpp::get_logger("GlobalReflectorTracker"),
              "Global Reflector Tracker destroyed");
}

void GlobalReflectorTracker::update(
    const std::vector<DetectedReflector> &detected_reflectors,
    const transforms::Rigid3d &laser_to_world, int64_t timestamp) {
  current_timestamp_ = timestamp;
  std::vector<int> matched_trackers;

  // 1. Data association: match detected reflectors with existing trackers
  for (const auto &detected : detected_reflectors) {
    // Filter out low confidence detections
    // TODO:
    // 检测置信度评分，暂时搁置，因为没有进行有效的圆度拟合与线性度融合评分 if
    // (detected.confidence < config_.min_confidence_to_track) {
    //     continue;
    // }

    // Transform to world frame
    Point global_position = transformToGlobal(detected.center, laser_to_world);

    // Find best match
    auto match = findBestMatch(global_position, tracked_reflectors_, true);

    if (match.tracker_id != -1) {
      // Update existing tracker，包括更新跟踪器的状态
      updateTrackedReflector(tracked_reflectors_[match.tracker_id], detected,
                             global_position, timestamp);
      matched_trackers.push_back(match.tracker_id);
    } else {
      // Create new tracker,
      // 这里没找到就分配新ID了，可能不太对，应该先分配资源状态，
      // 然后被状态机更新确认后，再分配ID
      auto new_tracker = createTrackedReflector(detected, global_position,
                                                next_global_id_++, timestamp);
      tracked_reflectors_.push_back(new_tracker);
      matched_trackers.push_back(new_tracker.global_id);
    }
  }
  // 检查是否存在指向相同ID的检测结果：
  for(int i=0; i < matched_trackers.size(); i++) {
    for(int j=i+1; j < matched_trackers.size(); j++) {
        if(matched_trackers[i]==matched_trackers[j] && matched_trackers[i] != -1) {
          RCLCPP_ERROR(rclcpp::get_logger("GlobalReflectorTracker"),
                      "两个反光柱(idx:%d, idx:%d)匹配了相同的TRACKING_ID: %d", i,j, matched_trackers[i]);
        } 
    }
  }
  auto found_detected = [&](int tracked_id)->bool {
    bool ret = false;
    int count = 0;
    for(const auto detect_id : matched_trackers) {
      if(tracked_id == detect_id) {
        ret = true;
        count++;
      }
    }
    if(count > 1) {
      RCLCPP_ERROR(rclcpp::get_logger("GlobalReflectorTracker"),
                   "跟踪器中的反光柱id: %d 被%d个反光柱同时匹配到", tracked_id, count);
    }
    return ret;
  };

  // 2. Update unmatched trackers
  for (size_t i = 0; i < tracked_reflectors_.size(); ++i) {
    if (!found_detected(tracked_reflectors_[i].global_id)) {
      tracked_reflectors_[i].consecutive_misses++;
      tracked_reflectors_[i].consecutive_detections = 0;
    }

    // Update state machine
    updateState(tracked_reflectors_[i], timestamp);
  }

  // 3. Cleanup expired trackers (only TENTATIVE state)
  cleanupExpiredReflectors(timestamp);
}

GlobalReflectorTracker::MatchResult GlobalReflectorTracker::findBestMatch(
    const Point &global_position,
    const std::vector<TrackedReflector> &tracked_reflectors,
    bool include_inactive) const {
  MatchResult result;
  result.tracker_id = -1;
  result.distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < tracked_reflectors.size(); ++i) {
    const auto &tracker = tracked_reflectors[i];

    // Skip INACTIVE state (if not including)
    if (!include_inactive && tracker.state == TrackedReflector::INACTIVE) {
      continue;
    }

    // Calculate matching distance (considering uncertainty)
    double distance = global_position.distanceTo(tracker.filtered_position);

    // Use different matching thresholds based on state
    double threshold = config_.match_distance_threshold;
    if (tracker.state == TrackedReflector::INACTIVE) {
      threshold = config_.match_distance_inactive;
    } else if (tracker.state == TrackedReflector::CONFIRMED) {
      // CONFIRMED state considers position uncertainty
      threshold += tracker.position_std_dev * 2.0;
    }

    if (distance < threshold && distance < result.distance) {
      result.tracker_id = i;
      result.distance = distance;
    }
  }

  return result;
}

void GlobalReflectorTracker::updateTrackedReflector(
    TrackedReflector &tracker, const DetectedReflector &detected,
    const Point &global_position, int64_t timestamp) {
  // Update basic information
  tracker.global_position = global_position;
  tracker.latest_detection = detected;
  tracker.last_detection_time = timestamp;

  // Update statistics
  tracker.total_detection_count++;
  tracker.consecutive_detections++;
  tracker.consecutive_misses = 0;

  // Add detection record to history
  TrackedReflector::DetectionRecord record;
  record.timestamp = timestamp;
  record.position = global_position;
  record.confidence = detected.confidence;
  tracker.detection_history.push_back(record);

  // Update position filter
  // 更新反光柱位置和位置偏差，和半径；实际上是对
  updatePositionFilter(tracker, global_position);

  // Update confidence
  // 滑窗检测器连续性检测，来更新tracker中上一次检测记录的置信度
  updateConfidence(tracker, detected.confidence);

  // Update state machine
  updateState(tracker, timestamp);
}

TrackedReflector GlobalReflectorTracker::createTrackedReflector(
    const DetectedReflector &detected, const Point &global_position,
    int global_id, int64_t timestamp) {
  TrackedReflector tracker;

  // Basic information
  tracker.global_id = global_id;
  tracker.global_position = global_position;
  tracker.filtered_position = global_position;
  tracker.latest_detection = detected;

  // State
  tracker.state = TrackedReflector::TENTATIVE;

  // Detection history
  TrackedReflector::DetectionRecord record;
  record.timestamp = timestamp;
  record.position = global_position;
  record.confidence = detected.confidence;
  tracker.detection_history.push_back(record);

  // Statistics
  tracker.total_detection_count = 1;
  tracker.consecutive_detections = 1;
  tracker.consecutive_misses = 0;
  tracker.last_detection_time = timestamp;
  tracker.first_detection_time = timestamp;

  // Position filtering
  tracker.position_std_dev = config_.min_std_dev;
  tracker.position_variance = config_.min_std_dev * config_.min_std_dev;

  // Confidence
  tracker.accumulated_confidence = detected.confidence;
  tracker.avg_confidence = detected.confidence;

  // Geometric properties
  tracker.diameter = detected.diameter;
  tracker.diameter_std_dev = 0.0;

  // Visualization
  tracker.visualization_color_id = global_id % 10; // Cycle through 10 colors

  return tracker;
}

void GlobalReflectorTracker::updatePositionFilter(
    TrackedReflector &tracker, const Point &measured_position) {
  double alpha = config_.position_filter_alpha;

  // EMA position filtering
  tracker.filtered_position.x =
      alpha * measured_position.x + (1 - alpha) * tracker.filtered_position.x;
  tracker.filtered_position.y =
      alpha * measured_position.y + (1 - alpha) * tracker.filtered_position.y;

  // Update uncertainty estimation
  updatePositionUncertainty(tracker, measured_position);

  // Diameter filtering
  tracker.diameter =
      config_.diameter_filter_alpha * tracker.latest_detection.diameter +
      (1 - config_.diameter_filter_alpha) * tracker.diameter;
}

void GlobalReflectorTracker::updatePositionUncertainty(
    TrackedReflector &tracker, const Point &measured_position) {
  // Calculate measurement error
  double dx = measured_position.x - tracker.filtered_position.x;
  double dy = measured_position.y - tracker.filtered_position.y;
  double squared_error = dx * dx + dy * dy;

  if (tracker.total_detection_count == 1) {
    // First detection: initialize variance
    tracker.position_variance = squared_error;
  } else {
    // EMA variance update
    double beta = config_.position_filter_beta;
    tracker.position_variance =
        beta * squared_error + (1 - beta) * tracker.position_variance;
  }

  // Calculate standard deviation
  tracker.position_std_dev = std::sqrt(tracker.position_variance);

  // Clamp standard deviation to range
  tracker.position_std_dev =
      std::max(config_.min_std_dev,
               std::min(config_.max_std_dev, tracker.position_std_dev));
}

void GlobalReflectorTracker::updateConfidence(TrackedReflector &tracker,
                                              double detection_confidence) {
  // Accumulate confidence
  tracker.accumulated_confidence += detection_confidence;

  // Calculate average confidence
  tracker.avg_confidence =
      tracker.accumulated_confidence / tracker.total_detection_count;

  // Adjust confidence based on position uncertainty
  double uncertainty_penalty = std::min(
      1.0, tracker.position_std_dev / config_.match_distance_threshold);
  double adjusted_confidence =
      tracker.avg_confidence * (1.0 - 0.3 * uncertainty_penalty);

  // Boost confidence if continuity check passes
  // 连续性检测, 如果为连续性检测，调整增加置信度
  if (checkContinuity(tracker)) {
    adjusted_confidence = std::min(1.0, adjusted_confidence + 0.2);
  }

  tracker.latest_detection.confidence = adjusted_confidence;
}

bool GlobalReflectorTracker::checkContinuity(TrackedReflector &tracker) const {
  // Remove detection records outside the time window
  // 检测当前时刻滑窗时间起点，以维护滑窗队列
  int64_t window_start =
      current_timestamp_ - int64_t(config_.confirm_time_window * 1e9);

  while (!tracker.detection_history.empty() &&
         tracker.detection_history.front().timestamp < window_start) {
    tracker.detection_history.pop_front();
  }

  // Check detection count in window
  size_t detections_in_window = tracker.detection_history.size();

  return detections_in_window >=
         static_cast<size_t>(config_.min_detections_in_window);
}

void GlobalReflectorTracker::updateState(TrackedReflector &tracker,
                                         int64_t current_time) {
  // 对于匹配到的反光柱检测时间差为0.0； 对于没有匹配到的反光柱计算与上一次检测时间差，用于判断反光柱真实状态
  // 以及是否处于非激活状态
  double time_since_last_detection = (current_time - tracker.last_detection_time) * 1e-9;

  switch (tracker.state) {
  case TrackedReflector::TENTATIVE:
    // Check if continuity condition is met
    if (checkContinuity(tracker)) {
      tracker.state = TrackedReflector::CONFIRMED;
      RCLCPP_INFO(
          rclcpp::get_logger("GlobalReflectorTracker"),
          "✓ Reflector %d confirmed as real reflector (detections: %zu)",
          tracker.global_id, tracker.detection_history.size());
    }
    // Reset if not detected for a long time
    else if (time_since_last_detection > config_.confirm_time_window * 2) {
      tracker.detection_history.clear();
      tracker.consecutive_misses = 0;
    }
    break;

  case TrackedReflector::CONFIRMED:
    // Check if should enter INACTIVE state
    if (time_since_last_detection > config_.inactive_timeout) {
      tracker.state = TrackedReflector::INACTIVE;
      RCLCPP_INFO(rclcpp::get_logger("GlobalReflectorTracker"),
                  "⚠ Reflector %d entered INACTIVE state", tracker.global_id);
    }
    break;

  case TrackedReflector::INACTIVE:
    // Check if should reactivate; 由于匹配上的反光柱时间为0;所以立刻进入确认状态
    if (time_since_last_detection <= config_.inactive_timeout) {
      tracker.state = TrackedReflector::CONFIRMED;
      RCLCPP_INFO(rclcpp::get_logger("GlobalReflectorTracker"),
                  "↻ Reflector %d reactivated", tracker.global_id);
    }
    break;
  }
}

void GlobalReflectorTracker::cleanupExpiredReflectors(int64_t current_time) {
  auto it = tracked_reflectors_.begin();
  while (it != tracked_reflectors_.end()) {
    double time_since_last_detection = (current_time - it->last_detection_time) * 1e-9;

    bool should_remove = false;

    // Remove TENTATIVE reflectors that haven't been confirmed
    // 超过两倍时间窗没有检测，进行动态过滤清除，认为其是无效landmark
    if (it->state == TrackedReflector::TENTATIVE) {
      if (time_since_last_detection > config_.confirm_time_window * 2) {
        should_remove = true;
      }
    }
    // Remove INACTIVE reflectors that exceeded max_inactive_time
    // 不清除INACTIVE状态的反光柱
    // else if (it->state == TrackedReflector::INACTIVE) {
    //     if (time_since_last_detection > config_.max_inactive_time) {
    //         should_remove = true;
    //         RCLCPP_INFO(rclcpp::get_logger("GlobalReflectorTracker"),
    //                    "✗ Reflector %d removed (inactive too long)",
    //                    it->global_id);
    //     }
    // }

    if (should_remove) {
      it = tracked_reflectors_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<TrackedReflector>
GlobalReflectorTracker::getConfirmedReflectors() const {
  std::vector<TrackedReflector> confirmed;
  for (const auto &tracker : tracked_reflectors_) {
    if (tracker.state == TrackedReflector::CONFIRMED) {
      confirmed.push_back(tracker);
    }
  }
  return confirmed;
}

std::vector<TrackedReflector>
GlobalReflectorTracker::getAllActiveReflectors() const {
  std::vector<TrackedReflector> active;
  for (const auto &tracker : tracked_reflectors_) {
    if (tracker.state == TrackedReflector::CONFIRMED ||
        tracker.state == TrackedReflector::TENTATIVE) {
      active.push_back(tracker);
    }
  }
  return active;
}

std::vector<TrackedReflector>
GlobalReflectorTracker::getAllTrackedReflectors() const {
  return tracked_reflectors_;
}

size_t GlobalReflectorTracker::getConfirmedReflectorCount() const {
  size_t count = 0;
  for (const auto &tracker : tracked_reflectors_) {
    if (tracker.state == TrackedReflector::CONFIRMED) {
      count++;
    }
  }
  return count;
}

void GlobalReflectorTracker::reset() {
  tracked_reflectors_.clear();
  next_global_id_ = 0;
  current_timestamp_ = 0.0;
  RCLCPP_INFO(rclcpp::get_logger("GlobalReflectorTracker"),
              "Global Reflector Tracker reset");
}

Point GlobalReflectorTracker::transformToGlobal(
    const Point &center_point,
    const transforms::Rigid3d &laser_to_world) const {
  // Transform point from laser frame to world frame
  Eigen::Vector3d laser_vec(center_point.x, center_point.y, 0.0);
  Eigen::Vector3d world_vec = laser_to_world * laser_vec;

  Point global_pos;
  global_pos.x = world_vec.x();
  global_pos.y = world_vec.y();

  return global_pos;
}

} // namespace amr_reflector_noise_handling