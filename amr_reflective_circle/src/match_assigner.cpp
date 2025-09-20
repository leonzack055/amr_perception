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
    : node_(node){
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
