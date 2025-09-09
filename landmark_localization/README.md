# Landmark Localization Package

## 使用方法

### 1. 启动节点（必选参数）

```bash
ros2 launch landmark_localization landmark_localization.launch.py pbstream_file:=/path/to/your/map.pbstream
```

### 2. 启动参数（可选参数）

```bash
ros2 launch landmark_localization landmark_localization.launch.py \
  pbstream_file:=/path/to/your/map.pbstream \
  scan_topic:=/your/scan/topic \
  lidar_frame:=your_lidar_frame \
  matching_threshold:= your_matching_threshold \
  tf_time_tolerance:= your_tf_time_tolerance
```

## 功能特性

- 从 `.pbstream` 文件中加载地标地图
- 订阅激光雷达数据（`sensor_msgs/msg/LaserScan`）
- 发布地标检测结果（`cartographer_ros_msgs/msg/LandmarkList`）
- 可选发布可视化标记（`visualization_msgs/msg/MarkerArray`）
- 支持自定义匹配阈值、TF 时间容差等参数

## 订阅的话题

- `scan_topic`（默认为 `/scan`）：激光雷达数据，类型为 `sensor_msgs/msg/LaserScan`

## 发布的话题

- `/landmark`：地标列表，类型为 `cartographer_ros_msgs/msg/LandmarkList`
- `/landmark_localization_markers`：可视化标记数组，类型为 `visualization_msgs/msg/MarkerArray`（仅在 `publish_visualization` 为 `true` 时发布）
  