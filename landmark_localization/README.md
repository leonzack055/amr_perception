# Landmark Localization Package

## 功能特性

- 从 `.pbstream` 文件中加载地标地图
- 订阅激光雷达数据（`sensor_msgs/msg/LaserScan`）
- 发布地标检测结果（`cartographer_ros_msgs/msg/LandmarkList`）
- 可选发布可视化标记（`visualization_msgs/msg/MarkerArray`）
- 支持自定义匹配阈值、TF 时间容差等参数

## 启动参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `pbstream_file` | `/path/to/your/map.pbstream` | 地图文件路径 |
| `scan_topic` | `/scan` | 激光雷达话题 |
| `matching_threshold` | `0.2` | 地标匹配阈值（米） |
| `tf_time_tolerance` | `0.05` | TF 时间容差（秒） |
| `lidar_frame` | `laser` | LiDAR 坐标系名称 |

## 节点配置

节点启动时会加载以下参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pbstream_file` | string | 由启动参数传入 | 地图文件路径 |
| `lidar_frame` | string | `laser` | LiDAR 坐标系 |
| `map_frame` | string | `map` | 地图坐标系 |
| `matching_threshold` | double | `0.2` | 地标匹配阈值 |
| `scan_topic` | string | `/scan` | 激光雷达话题 |
| `landmark_topic` | string | `/landmark` | 地标发布话题 |
| `publish_visualization` | bool | `true` | 是否发布可视化标记 |
| `visualization_topic` | string | `/landmark_localization_markers` | 可视化标记话题 |
| `use_simulation_params` | bool | `false` | 是否使用仿真参数 |
| `tf_time_tolerance` | double | `0.05` | TF 时间容差 |

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

## 订阅的话题

- `scan_topic`（默认为 `/scan`）：激光雷达数据，类型为 `sensor_msgs/msg/LaserScan`

## 发布的话题

- `/landmark`：地标列表，类型为 `cartographer_ros_msgs/msg/LandmarkList`
- `/landmark_localization_markers`：可视化标记数组，类型为 `visualization_msgs/msg/MarkerArray`（仅在 `publish_visualization` 为 `true` 时发布）

## 注意事项

1. 确保提供的 `.pbstream` 文件路径正确且可访问。
2. 激光雷达坐标系（`lidar_frame`）必须与 TF 树中的坐标系一致。
3. 若在仿真环境中使用，请设置 `use_simulation_params` 为 `true`。