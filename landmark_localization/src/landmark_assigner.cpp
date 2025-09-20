#include "landmark_localization/landmark_assigner.hpp"

namespace landmark_localization
{
LandmarkAssigner::LandmarkAssigner()
: search_range_(10.0), match_threshold_(0.2), id_counter_(0),
  base_to_laser_transform_available_(false),
  laser_frame_(""), last_pose_time_ns_(0)
{
}


void LandmarkAssigner::update_tracked_pose(const transforms::Rigid3d & pose, int64_t time_ns)
{
  tracked_pose_ = pose;
  last_pose_time_ns_ = time_ns;
}

std::vector<int> LandmarkAssigner::matchCurrentToHistory(
  const std::vector<Detection> & current_detections)
{
  std::vector<int> matched_detections(current_detections.size(), -1);
  std::vector<int> optimized_bar_ids;
  std::vector<int> tracked_bar_ids;
  // 1. 从机器人所在位姿置，找到已经优化过的反光柱
  transforms::Rigid3d robot_pose = tracked_pose_;
  for (const auto & reflector_bar : reflector_bars_) {
    auto reflector_bar_pose = reflector_bar.second.g_detection_.pose.pose;
    double distance =
      (robot_pose.translation() - reflector_bar_pose.translation()).norm();
    if (distance < search_range_) {
      if (reflector_bar.second.optimized_) {
        optimized_bar_ids.push_back(reflector_bar.first);
      } else {
        tracked_bar_ids.push_back(reflector_bar.first);
      }
    }
  }
  // 2. 根据当前查找结果更新反光柱
  for (int id = 0; id < current_detections.size(); id++) {
    double opt_distance = std::numeric_limits<double>::max();
    double track_distance = std::numeric_limits<double>::max();
    int opt_id = -1;
    int track_id = -1;
    // 2.1 从优化过的反光柱中，找到最近的反光柱
    transforms::Rigid3d current_detect_landmark = current_detections[id].pose.pose;
    for (const auto & optimized_bar_id : optimized_bar_ids) {
      // 2.1.1 计算当前反光柱与优化过的反光柱的距离
      auto reflector_bar_pose = reflector_bars_[optimized_bar_id].g_detection_.pose.pose;
      double d = (current_detect_landmark.translation() -
        reflector_bar_pose.translation())
        .norm();
      if (d < opt_distance) {
        opt_distance = d;
        opt_id = optimized_bar_id;
      }
    }
    // 2.2 从未优化过的反光柱中，找到最近的反光柱
    for (const auto & tracked_bar_id : tracked_bar_ids) {
      auto reflector_bar_pose = reflector_bars_[tracked_bar_id].g_detection_.pose.pose;
      double d = (current_detect_landmark.translation() -
        reflector_bar_pose.translation())
        .norm();
      if (d < track_distance) {
        track_distance = d;
        track_id = tracked_bar_id;
      }
    }
    // 2.3 对比优化过的反光柱和未优化过的反光柱，选择最近的反光柱
    if (opt_distance < match_threshold_) {
      matched_detections[id] = opt_id;
    } else if (track_distance < match_threshold_) {
      matched_detections[id] = track_id;
    }
    // TODO： 只用局部里程不使用全局
    //  if (track_distance < match_threshold_) {
    //   matched_detections[id] = track_id;
    // }
  }
  return matched_detections;
}

bool LandmarkAssigner::time_check(int64_t time1, int64_t time2, double threshold) const
{
  return std::abs(time1 - time2) < threshold * 1e9;
}


bool LandmarkAssigner::isLandmarkDetectorOK(int64_t detect_timestamp) const
{
  if (base_to_laser_transform_available_ && time_check(last_pose_time_ns_, detect_timestamp, 0.3)) {
    return true;
  }
  return false;
}


Eigen::Quaterniond LandmarkAssigner::CorrectOrientationToLaser()
{
  Eigen::Quaterniond pose_quat = tracked_pose_.rotation();
  Eigen::Quaterniond corrected_landmark_orient = Eigen::Quaterniond::Identity();
  corrected_landmark_orient = pose_quat.conjugate().normalized();
  return corrected_landmark_orient;
}


void LandmarkAssigner::update_landmarks(
  const std::map<int, transforms::Rigid3d> & landmarks, int64_t landmark_time)
{
  for (const auto & landmark_marker: landmarks) {
    // 处理反光柱位姿
    std::lock_guard<std::mutex> lock(reflector_bars_mutex_);
    Eigen::Quaterniond g_orientation = Eigen::Quaterniond(
      landmark_marker.second.rotation().w(), landmark_marker.second.rotation().x(),
      landmark_marker.second.rotation().y(), landmark_marker.second.rotation().z());
    int id = landmark_marker.first;
    // TODO: check g_orientation　是否Identity
    if (!g_orientation.isApprox(Eigen::Quaterniond::Identity())) {
      // RCLCPP_WARN(node_->get_logger(), "landmark %d 不是全局坐标位姿", id);
    }

    if (reflector_bars_.find(id) == reflector_bars_.end()) {
      printf(
        "landmark %d 没有经过匹配器就进行了发布，检查代码!", id);
    }
    if (reflector_bars_[id].id_ == id) {
      auto old_global_pose = reflector_bars_[id].g_detection_.pose.pose;
      auto new_global_pose = landmark_marker.second;
      auto relative_pose = old_global_pose.inverse() * new_global_pose;
      auto yaw = transforms::GetYaw(relative_pose);
      if (relative_pose.translation().norm() > 0.1) {
        std::cout << "landmark " << id << " Carto优化后位姿更新, yaw: " << yaw
                  << "，距离: " << relative_pose.translation().norm();
      }
    }
    // 更新表中全局landmarks位姿　
    reflector_bars_[id].g_detection_.pose.pose = landmark_marker.second;
    reflector_bars_[id].g_detection_.pose.header = laser_frame_;
    reflector_bars_[id].time_ = landmark_time;
    reflector_bars_[id].optimized_ = true;
  }
}


std::vector<Detection> LandmarkAssigner::getGlobalDetections(
  const std::vector<Detection> & current_detections)
{
  std::vector<Detection> global_detections;
  if (!base_to_laser_transform_available_) {
    return global_detections;
  }

  global_detections = current_detections;
  // 转换到全局坐标
  for (int i = 0; i < current_detections.size(); i++) {
    transforms::Rigid3d robot_pose = tracked_pose_;
    transforms::Rigid3d laser_to_landmark = current_detections[i].pose.pose;
    transforms::Rigid3d current_global_landmark =
      robot_pose * base_to_laser_transform_ * laser_to_landmark;
    // 2. 设定全局的landmark的位姿为Identity
    transforms::Rigid3d global_landmark = transforms::Rigid3d(
      current_global_landmark.translation(), Eigen::Quaterniond::Identity());
    // TODO: 设定全局的landmark的位姿为Identity
    global_detections[i].pose.pose = global_landmark;
  }
  return global_detections;
}

std::vector<ReflectorBar> LandmarkAssigner::assignLandmarkToReflectorBar(
  const std::vector<Detection> & current_detections)
{
  std::lock_guard<std::mutex> lock(reflector_bars_mutex_);
  // 1. 转化为全局坐标系下的位姿
  std::vector<Detection> global_detections =
    getGlobalDetections(current_detections);
  // 2. 查找匹配的历史检测
  std::vector<int> matched_detections =
    matchCurrentToHistory(global_detections);
  // std::vector<Detection> fused_global_detections = global_detections;
  // 3. 分配新ID
  for (int idx = 0; idx < matched_detections.size(); idx++) {
    if (matched_detections[idx] == -1) {
      transforms::Rigid3d global_landmark = global_detections[idx].pose.pose;
      // 分配新ID，并进行跟踪记录
      ReflectorBar new_reflectorbar;
      new_reflectorbar.age_ = 0;
      new_reflectorbar.g_detection_ = global_detections[idx];
      // 维持全局位姿Identity
      new_reflectorbar.time_ = last_pose_time_ns_;
      new_reflectorbar.id_ = id_counter_++;
      new_reflectorbar.id_str_ = std::to_string(new_reflectorbar.id_);
      new_reflectorbar.optimized_ = false;
      reflector_bars_[new_reflectorbar.id_] = new_reflectorbar;
      matched_detections[idx] = new_reflectorbar.id_;
      std::cout << "分配新ID: " << new_reflectorbar.id_ << std::endl;
    } else {
      // std::cout << "区配旧ID: " << matched_detections[idx] << std::endl;
    }
  }
  // 4. 将全局坐标下的位姿，转换为激光坐标系下的位姿
  std::vector<ReflectorBar> local_detections(global_detections.size());
  assert(matched_detections.size() == global_detections.size());
  for (int idx = 0; idx < matched_detections.size(); idx++) {
    if (matched_detections[idx] == -1) {
      std::cerr << "当前匹配还有没分匹ID的landmark" << std::endl;
      continue;
    }
    transforms::Rigid3d robot_pose = tracked_pose_;
    transforms::Rigid3d global_landmark = global_detections[idx].pose.pose;
    transforms::Rigid3d laser_landmark = base_to_laser_transform_.inverse() *
      robot_pose.inverse() * global_landmark;
    // 填充检测结果
    local_detections[idx].age_ = reflector_bars_[matched_detections[idx]].age_;
    local_detections[idx].g_detection_ = global_detections[idx];
    local_detections[idx].g_detection_.pose.pose = laser_landmark;
    local_detections[idx].id_ = matched_detections[idx];
    local_detections[idx].id_str_ = std::to_string(matched_detections[idx]);
  }
  return local_detections;
}

}  // namespace landmark_localization