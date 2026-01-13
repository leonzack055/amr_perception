# Landmark Localization Package

## 使用方法

### 1. 基本启动

```bash
ros2 launch landmark_localization landmark_localization.launch.py pbstream_file:=/path/to/your/map.pbstream
```

### 2. 完整参数启动

```bash
ros2 launch landmark_localization landmark_localization.launch.py \
  pbstream_file:=/path/to/your/map.pbstream \
  map_frame:=map \
  base_frame:=base_link \
  odom_frame:=odom \
  matching_threshold:=0.2 \
  tf_time_tolerance:=0.05 \
  intensity_threshold_use:=1600 \
  min_landmarks_for_pose:=3 \
  use_lidar1:=true \
  scan1_topic:=/scan \
  use_lidar2:=true \
  scan2_topic:=/rear_scan \
  use_combine:=false \
  use_calculate_filter:=false \
  filter_num:=5 \
  log_level:=info
```

## 参数说明

### 必选参数
- `pbstream_file`: pbstream地图文件路径（默认：`/path/to/your/map.pbstream`）

### 可选参数
- `map_frame`: 地图坐标系ID（默认：`map`）
- `base_frame`: 机器人基座坐标系ID（默认：`base_link`）
- `odom_frame`: 里程计坐标系ID（默认：`odom`）
- `matching_threshold`: 地标匹配阈值，单位：米（默认：`0.2`）
- `tf_time_tolerance`: TF时间容差，单位：秒（默认：`0.05`）
- `intensity_threshold_use`: 强度阈值（默认：`1600`）
- `min_landmarks_for_pose`: 计算机器人位姿所需的最少匹配地标数量（默认：`3`）

### 激光雷达参数
- `use_lidar1`: 是否使用激光雷达1（默认：`true`）
- `scan1_topic`: 激光雷达1扫描数据话题（默认：`/scan`）
- `use_lidar2`: 是否使用激光雷达2（默认：`true`）
- `scan2_topic`: 激光雷达2扫描数据话题（默认：`/rear_scan`）
- `use_combine`: 是否对激光雷达1和2进行联合处理（默认：`false`）

### 滤波参数
- `use_calculate_filter`: 是否对位姿计算使用滤波（默认：`false`）
- `filter_num`: 滤波次数：相同地标组合需要的连续次数（默认：`5`）

### 日志参数
- `log_level`: 日志级别（debug, info, warn, error, fatal）（默认：`info`）

## 功能特性

- 从 `.pbstream` 文件中加载地标地图
- 支持双激光雷达数据订阅（`sensor_msgs/msg/LaserScan`）
- 发布地标检测结果（`cartographer_ros_msgs/msg/LandmarkList`）
- 发布可视化标记（`visualization_msgs/msg/MarkerArray`）
- 支持自定义匹配阈值、TF 时间容差等参数
- 支持地标匹配滤波和位姿计算优化

## 订阅的话题

- `/scan`：激光雷达1数据，类型为 `sensor_msgs/msg/LaserScan`
- `/rear_scan`：激光雷达2数据，类型为 `sensor_msgs/msg/LaserScan`

## 发布的话题

- `/landmark`：地标列表，类型为 `cartographer_ros_msgs/msg/LandmarkList`
- `/landmark_localization_markers`：可视化标记数组，类型为 `visualization_msgs/msg/MarkerArray`
- `/global_pose_qr`：全局位姿，类型为 `geometry_msgs/msg/PoseWithCovarianceStamped`
- `/initial_pose`：初始位姿，类型为 `geometry_msgs/msg/PoseWithCovarianceStamped`
  