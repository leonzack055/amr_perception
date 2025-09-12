/**
 * @file match_assigner.cpp
 * @author Leon
 * @brief　反光柱，匹配分配器；持续获取，当前车量全局位姿，以全局位姿保存，反光柱的位姿;
 * 2. 持续获取优化后的反光柱位姿，以更新全局反光柱位姿
 * 3.
 * 以车量位姿为基准，进行反光柱的匹配获取id分配；当存在全局位姿时，以车辆中心为半径，进行查找可视范围内的反光柱
 * 进行匹配分配
 * @version 0.1
 * @date 2023-08-10
 *
 */

#include "match_assigner.hpp"

MatchAssigner::MatchAssigner(const rclcpp::Node::SharedPtr& node)
    : node_(node), search_range_(10.0), match_threshold_(0.2) {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
      node_->get_clock(), tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
      node_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // 订阅车量位姿
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tracked_pose", 10,
      std::bind(&MatchAssigner::pose_callback, this, std::placeholders::_1));
  //     // 订阅反光柱位姿
  landmark_sub_ =
      node_->create_subscription<visualization_msgs::msg::MarkerArray>(
          "/landmark_poses_list", 10,
          std::bind(&MatchAssigner::landmark_callback, this,
                    std::placeholders::_1));

  // 新增：创建全局反光柱可视化发布器
  global_reflector_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/global_reflector_bars", 10);

  // 新增：创建定时器，定期发布全局反光柱可视化
  visualization_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500),  // 每500ms发布一次
      std::bind(&MatchAssigner::publishGlobalReflectorBars, this));

  RCLCPP_INFO(node_->get_logger(), "MatchAssigner 初始化完成");
}

bool MatchAssigner::isLandmarkDetectorOK() {
  if (pose_msg_ == nullptr) {
    RCLCPP_WARN(node_->get_logger(),
                "pose_msg_ is nullptr, 还没收到/tracked_pose消息");
    return false;
  }
  return true;
}

void MatchAssigner::setLaserFrame(const std::string& laser_frame) {
  if (laser_frame_.empty()) {
    laser_frame_ = laser_frame;
  }
}

geometry_msgs::msg::PoseStamped MatchAssigner::fusePoses(
    const geometry_msgs::msg::PoseStamped& pose1,
    const geometry_msgs::msg::PoseStamped& pose2, double weight1,
    double weight2) {
  geometry_msgs::msg::PoseStamped fused_pose;
  double total_weight = weight1 + weight2;

  // 融合位置
  fused_pose.pose.position.x =
      (pose1.pose.position.x * weight1 + pose2.pose.position.x * weight2) /
      total_weight;
  fused_pose.pose.position.y =
      (pose1.pose.position.y * weight1 + pose2.pose.position.y * weight2) /
      total_weight;
  fused_pose.pose.position.z = 0.0;  // 假设z坐标为0

  // １. 维持全局坐标 Identity
  fused_pose.pose.orientation.x = 0.0;
  fused_pose.pose.orientation.y = 0.0;
  fused_pose.pose.orientation.z = 0.0;
  fused_pose.pose.orientation.w = 1.0;

  return fused_pose;
}

std::vector<Detection> MatchAssigner::getGlobalDetections(
    const std::vector<Detection>& current_detections) {
  std::vector<Detection> global_detections;
  global_detections = current_detections;

  if (!base_to_laser_transform_available_) {
    try {
      tf2::Duration timeout(tf2::durationFromSec(0.1));
      geometry_msgs::msg::TransformStamped base2laser_tf =
          tf_buffer_->lookupTransform("base_link", laser_frame_,
                                      ::rclcpp::Time(0.), timeout);
      base_to_laser_transform_ = transforms::ToRigid3d(base2laser_tf);
      base_to_laser_transform_available_ = true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "TF Buffer Transform exception: " << ex.what());
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "完成baselink到laser的坐标获取");
  }

  // 转换到全局坐标
  for (int i = 0; i < current_detections.size(); i++) {
    transforms::Rigid3d robot_pose = transforms::ToRigid3d(pose_msg_->pose);
    transforms::Rigid3d laser_to_landmark =
        transforms::ToRigid3d(current_detections[i].pose.pose);
    transforms::Rigid3d current_global_landmark =
        robot_pose * base_to_laser_transform_ * laser_to_landmark;
    // 2. 设定全局的landmark的位姿为Identity
    transforms::Rigid3d global_landmark = transforms::Rigid3d(
        current_global_landmark.translation(), Eigen::Quaterniond::Identity());
    // TODO: 设定全局的landmark的位姿为Identity
    global_detections[i].pose.pose =
        transforms::ToGeometryMsgPose(global_landmark);
  }
  return global_detections;
}

// 融合历史反光柱位姿
std::vector<Detection> MatchAssigner::updateTemporalFilter(
    const std::vector<Detection>& current_detections,
    const std::vector<int>& matched_bars) {
  std::vector<Detection> filtered_detections;
  for (int i = 0; i < matched_bars.size(); i++) {
    if (matched_bars[i] == -1) {
      filtered_detections.push_back(current_detections[i]);
      continue;
    }
    // 匹配到历史记录，融合结果
    geometry_msgs::msg::PoseStamped fused_pose =
        fusePoses(current_detections[i].pose,
                  reflector_bars_[matched_bars[i]].g_detection_.pose,
                  current_detections[i].confidence,
                  reflector_bars_[matched_bars[i]].g_detection_.confidence);
    // 融合置信度
    double fused_confidence =
        (current_detections[i].confidence,
         +reflector_bars_[matched_bars[i]].g_detection_.confidence * 0.8) /
        1.8;

    filtered_detections.push_back(
        {fused_pose, current_detections[i].diameter, fused_confidence,
         current_detections[i].translationW, current_detections[i].rotationW});
  }

  // 去除未匹配的局部历史记录，可能没有被后端更新，当成是误检
  // for (auto it = reflector_bars_.begin(); it != reflector_bars_.end(); ) {
  //   auto & [id, entry] = *it;
  //   if (entry.optimized_) {continue;}
  //   if (entry.age_ > 5) {
  //     it = reflector_bars_.erase(it);
  //   } else {
  //     entry.g_detection_.confidence *= 0.95;
  //     entry.age_ += 1;
  //     ++it;
  //   }
  // }

  return filtered_detections;
}

std::vector<ReflectorBar> MatchAssigner::assignLandmarkToReflectorBar(
    const std::vector<Detection>& current_detections) {
  std::lock_guard<std::mutex> lock(reflector_bars_mutex_);
  // 1. 转化为全局坐标系下的位姿
  std::vector<Detection> global_detections =
      getGlobalDetections(current_detections);
  // 2. 查找匹配的历史检测
  std::vector<int> matched_detections =
      matchCurrentToHistory(global_detections);
  // 3. 局部融合
  std::vector<Detection> fused_global_detections =
      updateTemporalFilter(global_detections, matched_detections);
  // std::vector<Detection> fused_global_detections = global_detections;
  // 3. 分配新ID
  for (int idx = 0; idx < matched_detections.size(); idx++) {
    if (matched_detections[idx] == -1) {
      transforms::Rigid3d global_landmark =
          transforms::ToRigid3d(fused_global_detections[idx].pose.pose);
      // 分配新ID，并进行跟踪记录
      ReflectorBar new_reflectorbar;
      new_reflectorbar.age_ = 0;
      new_reflectorbar.g_detection_ = fused_global_detections[idx];
      // 维持全局位姿Identity
      new_reflectorbar.time_ = fused_global_detections[idx].pose.header.stamp;
      new_reflectorbar.id_ = id_counter_++;
      new_reflectorbar.id_str_ = std::to_string(new_reflectorbar.id_);
      new_reflectorbar.optimized_ = false;
      reflector_bars_[new_reflectorbar.id_] = new_reflectorbar;
      matched_detections[idx] = new_reflectorbar.id_;
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "分配新ID: " << new_reflectorbar.id_);
    } else {
      RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1,
                                  "区配旧ID: " << matched_detections[idx]);
    }
  }
  // 4. 将全局坐标下的位姿，转换为激光坐标系下的位姿
  std::vector<ReflectorBar> local_detections(fused_global_detections.size());
  assert(matched_detections.size() == fused_global_detections.size());
  for (int idx = 0; idx < matched_detections.size(); idx++) {
    if (matched_detections[idx] == -1) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "当前匹配还有没分匹ID的landmark");
      continue;
    }
    transforms::Rigid3d robot_pose = transforms::ToRigid3d(pose_msg_->pose);
    transforms::Rigid3d global_landmark =
        transforms::ToRigid3d(fused_global_detections[idx].pose.pose);
    transforms::Rigid3d laser_landmark = base_to_laser_transform_.inverse() *
                                         robot_pose.inverse() * global_landmark;
    // 填充检测结果
    local_detections[idx].age_ = reflector_bars_[matched_detections[idx]].age_;
    local_detections[idx].g_detection_ = fused_global_detections[idx];
    local_detections[idx].g_detection_.pose.pose =
        transforms::ToGeometryMsgPose(laser_landmark);
    local_detections[idx].id_ = matched_detections[idx];
    local_detections[idx].id_str_ = std::to_string(matched_detections[idx]);
  }
  return local_detections;
}

Eigen::Quaterniond MatchAssigner::CorrectOrientationToLaser() {
  Eigen::Quaterniond pose_quat =
      transforms::ToEigen(pose_msg_->pose.orientation);
  Eigen::Quaterniond corrected_landmark_orient = Eigen::Quaterniond::Identity();
  corrected_landmark_orient = pose_quat.conjugate().normalized();
  return corrected_landmark_orient;
}

bool MatchAssigner::time_check(rclcpp::Time time1, rclcpp::Time time2,
                               double threshold) {
  return (time1 - time2).seconds() < threshold;
}

void MatchAssigner::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(reflector_bars_mutex_);
  pose_msg_ = msg;
}

std::vector<int> MatchAssigner::matchCurrentToHistory(
    const std::vector<Detection>& current_detections) {
  std::vector<int> matched_detections(current_detections.size(), -1);
  std::vector<int> optimized_bar_ids;
  std::vector<int> tracked_bar_ids;
  // 1. 从机器人所在位姿置，找到已经优化过的反光柱
  transforms::Rigid3d robot_pose = transforms::ToRigid3d(pose_msg_->pose);
  for (const auto& reflector_bar : reflector_bars_) {
    auto reflector_bar_pose =
        transforms::ToRigid3d(reflector_bar.second.g_detection_.pose.pose);
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
    transforms::Rigid3d current_detect_landmark =
        transforms::ToRigid3d(current_detections[id].pose.pose);
    for (const auto& optimized_bar_id : optimized_bar_ids) {
      // 2.1.1 计算当前反光柱与优化过的反光柱的距离
      auto reflector_bar_pose = transforms::ToRigid3d(
          reflector_bars_[optimized_bar_id].g_detection_.pose.pose);
      double d = (current_detect_landmark.translation() -
                  reflector_bar_pose.translation())
                     .norm();
      if (d < opt_distance) {
        opt_distance = d;
        opt_id = optimized_bar_id;
      }
    }
    // 2.2 从未优化过的反光柱中，找到最近的反光柱
    for (const auto& tracked_bar_id : tracked_bar_ids) {
      auto reflector_bar_pose = transforms::ToRigid3d(
          reflector_bars_[tracked_bar_id].g_detection_.pose.pose);
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

void MatchAssigner::landmark_callback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
  for (const auto& marker_msg : msg->markers) {
    // 处理反光柱位姿
    if (marker_msg.ns == "Landmarks" && marker_msg.header.frame_id == "map") {
      std::lock_guard<std::mutex> lock(reflector_bars_mutex_);
      Eigen::Quaterniond g_orientation = Eigen::Quaterniond(
          marker_msg.pose.orientation.w, marker_msg.pose.orientation.x,
          marker_msg.pose.orientation.y, marker_msg.pose.orientation.z);
      int id = marker_msg.id;
      // TODO: check g_orientation　是否Identity
      if (!g_orientation.isApprox(Eigen::Quaterniond::Identity())) {
        // RCLCPP_WARN(node_->get_logger(), "landmark %d 不是全局坐标位姿", id);
      }
      
      if (reflector_bars_.find(id) == reflector_bars_.end()) {
        RCLCPP_WARN(node_->get_logger(),
                    "landmark %d 没有经过匹配器就进行了发布，检查代码!", id);
      }
      if(reflector_bars_[id].id_ == id) {
        auto old_global_pose  = transforms::ToRigid3d(reflector_bars_[id].g_detection_.pose.pose);
        auto new_global_pose = transforms::ToRigid3d(marker_msg.pose);
        auto relative_pose = old_global_pose.inverse() * new_global_pose;
        auto yaw = transforms::GetYaw(relative_pose);
        if(relative_pose.translation().norm() > 0.1) {
          RCLCPP_INFO_STREAM(node_->get_logger(), "landmark " << id << " Carto优化后位姿更新, yaw: " << yaw 
            << "，距离: " << relative_pose.translation().norm());
        }
      }
      // 更新表中全局landmarks位姿　
      reflector_bars_[id].g_detection_.pose.pose = marker_msg.pose;
      reflector_bars_[id].g_detection_.pose.header = marker_msg.header;
      reflector_bars_[id].time_ = marker_msg.header.stamp;
      reflector_bars_[id].optimized_ = true;
    } else {
      RCLCPP_INFO_STREAM(node_->get_logger(), "landmarks 消息不对内容");
    }
  }
}

// 新增：发布全局反光柱可视化
void MatchAssigner::publishGlobalReflectorBars() {
  if (reflector_bars_.empty()) {
    return;  // 没有反光柱数据时不发布
  }

  visualization_msgs::msg::MarkerArray marker_array;
  rclcpp::Time current_time = node_->now();

  for (const auto& [id, reflector_bar] : reflector_bars_) {
    // 检查反光柱是否在搜索范围内
    if (pose_msg_ != nullptr) {
      transforms::Rigid3d robot_pose = transforms::ToRigid3d(pose_msg_->pose);
      transforms::Rigid3d reflector_pose =
          transforms::ToRigid3d(reflector_bar.g_detection_.pose.pose);
      double distance =
          (robot_pose.translation() - reflector_pose.translation()).norm();

      // 只显示在搜索范围内的反光柱
      if (distance > search_range_) {
        continue;
      }
    }

    // 添加圆柱体标记
    visualization_msgs::msg::Marker marker =
        createReflectorBarMarker(reflector_bar, "map");
    marker.header.stamp = current_time;
    marker_array.markers.push_back(marker);

    // 添加文本标记显示ID
    visualization_msgs::msg::Marker text_marker =
        createTextMarker(reflector_bar, "map");
    text_marker.header.stamp = current_time;
    marker_array.markers.push_back(text_marker);
  }

  // 发布可视化标记
  global_reflector_publisher_->publish(marker_array);
}

// 新增：创建反光柱标记
visualization_msgs::msg::Marker MatchAssigner::createReflectorBarMarker(
    const ReflectorBar& reflector_bar, const std::string& frame_id) {
  visualization_msgs::msg::Marker marker;

  // 设置基本属性
  marker.header.frame_id = frame_id;
  marker.ns = "global_reflector_bars";
  marker.id = reflector_bar.id_;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 设置位置和朝向
  marker.pose = reflector_bar.g_detection_.pose.pose;

  // 设置尺寸
  double diameter = reflector_bar.g_detection_.diameter;
  marker.scale.x = diameter;  // 直径
  marker.scale.y = diameter;  // 直径
  marker.scale.z = 0.2;       // 高度

  // 根据优化状态设置颜色
  if (reflector_bar.optimized_) {
    // 已优化的反光柱: 蓝色
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
  } else {
    // 跟踪中的反光柱：透明黄色
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
  }

  // 设置生命周期
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  return marker;
}

// 新增：创建文本标记显示ID
visualization_msgs::msg::Marker MatchAssigner::createTextMarker(
    const ReflectorBar& reflector_bar, const std::string& frame_id) {
  visualization_msgs::msg::Marker marker;

  // 设置基本属性
  marker.header.frame_id = frame_id;
  marker.ns = "reflector_ids";
  marker.id = reflector_bar.id_ + 10000;  // 避免与实际landmark id冲突
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 设置位置（在反光柱上方）
  marker.pose.position.x = reflector_bar.g_detection_.pose.pose.position.x;
  marker.pose.position.y = reflector_bar.g_detection_.pose.pose.position.y;
  marker.pose.position.z = reflector_bar.g_detection_.pose.pose.position.z +
                           0.3;  // 在反光柱上方0.3米
  marker.pose.orientation.w = 1.0;

  // 设置文本内容
  marker.text = "id:" + reflector_bar.id_str_;

  // 设置文本大小
  marker.scale.z = 0.2;  // 文本高度

  // 设置颜色（白色）
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // 设置生命周期
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  return marker;
}
