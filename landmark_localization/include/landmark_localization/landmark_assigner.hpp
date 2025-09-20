#ifndef LANDMARK_ASSIGNER_HPP_
#define LANDMARK_ASSIGNER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>
#include <vector>
#include <mutex>

#include "common/reflector_common.hpp"

/**
 * 1. 设定baselink_to_laser的坐标，并将标识位置为True
 * 2. 校验当前检测到的反光柱是否可用，isLandmarkDetectorOK()
 * 3. 如果可用，assignLandmarkToReflectorBar()
 * 外部设定参数
 * (1) search_range_：匹配搜索范围
 * (2) match_threshold_：匹配阈值
 * (3) base_to_laser_transform_：是否已获取baselink_to_laser的外参位姿
 * (4) tracked_pose_：当前tracked_pose全局位姿
 * (5) update_landmarks()：更新landmark的全局位姿
 */
namespace landmark_localization
{

class LandmarkAssigner
{
public:
  LandmarkAssigner();
  // 获取当前的检测到相对于激光坐标系下的landmarks的在map系下的全局位姿
  std::vector<Detection> getGlobalDetections(
    const std::vector<Detection> & current_detections);
  // 进行历史landmark_id匹配并分配新的id
  std::vector<ReflectorBar> assignLandmarkToReflectorBar(
    const std::vector<Detection> & current_detections);
  // 新增：修正检测方向，以global下的Identity进行矫正
  Eigen::Quaterniond CorrectOrientationToLaser();
  // 检查是否tf己经开启，并且当前trackted_pose与landmark检测时间不超过300ms
  bool isLandmarkDetectorOK(int64_t detect_timestamp) const;

  // 处理landmark检测的消息
  void update_landmarks(
    const std::map<int, transforms::Rigid3d> & landmarks, int64_t landmark_time);

  // 参数属性设定
  const std::string laserframe() const {return laser_frame_;}
  const double search_range() const {return search_range_;}
  const double match_threshold() const {return match_threshold_;}
  const transforms::Rigid3d & getTrackedPose() {return tracked_pose_;}
  const transforms::Rigid3d & getBase2LaserTrans() {return base_to_laser_transform_;}
  const int64_t last_time_ns() const {return last_pose_time_ns_;}

  LandmarkAssigner & search_range(const double & range) {search_range_ = range; return *this;}
  LandmarkAssigner & match_threshold(const double & threshold)
  {
    match_threshold_ = threshold; return *this;
  }
  LandmarkAssigner & laser_frame(const std::string & frame) {laser_frame_ = frame; return *this;}
  LandmarkAssigner & setBase2LaserTrans(
    const transforms::Rigid3d & transform)
  {
    base_to_laser_transform_ = transform; 
    base_to_laser_transform_available_ = true;
    return *this;
  }
  // 更新当前tracked_pose全局位姿的时间，以保证其连续可用
  void update_tracked_pose(const transforms::Rigid3d & pose, int64_t time_ns);

  const ReflectorBarMap & getReflectorBars() {return reflector_bars_;}

protected:
  bool time_check(int64_t time1, int64_t time2, double threshold) const;

private:
  // 与历史的landmark进行匹配，返回当前landmark的匹配索引和新ID
  std::vector<int> matchCurrentToHistory(
    const std::vector<Detection> & current_detections);

  // 内参
  double search_range_; // 匹配搜索范围
  double match_threshold_; // 匹配阈值
  int id_counter_ = 0;
  bool base_to_laser_transform_available_ = false; // 已获取baselink_to_laser的外参位姿
  transforms::Rigid3d base_to_laser_transform_; // baselink_to_laser的外参位姿
  std::string laser_frame_;

  ReflectorBarMap reflector_bars_; //反光柱的全局位姿表
  // mutex
  std::mutex reflector_bars_mutex_;

  transforms::Rigid3d tracked_pose_; // 跟踪的当前车体中心
  int64_t last_pose_time_ns_ = 0; // 跟踪车辆的时间戳
};

}  // namespace landmark_localization


#endif  // LANDMARK_ASSIGNER_HPP_
