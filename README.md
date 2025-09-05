# amr_perception

## 编译  

### colcon build --packages-up-to amr_perception --symlink-install

## 反光条检测的python版本

### ros2 launch amr_reflective_strip_recongnition yl.launch.py

## 反光条检测的c++版本

### ros2 launch amr_reflective_strip_recongnition_cpp reflector_detector.launch.py

## 反光柱检测的代码

### ros2 launch amr_reflective_circle reflector_detector_circle.launch.py



## 新增landmark功能

### 新增功能

对应接口：
- `MatchAssigner::publishGlobalReflectorBars()`：定时发布全局可视化。
- `MatchAssigner::createReflectorBarMarker(...)`：创建圆柱体标记。
- `MatchAssigner::createTextMarker(...)`：创建 ID 文本标记。
- `MatchAssigner::assignLandmarkToReflectorBar(...)`：主匹配/分配流程。
- `MatchAssigner::getGlobalDetections(...)`、`updateTemporalFilter(...)`、`matchCurrentToHistory(...)`：坐标转换、时序融合、历史匹配。

### 话题接口
- **发布**
  - `/global_reflector_bars` (`visualization_msgs/MarkerArray`)：全局反光柱可视化（圆柱 + id）。

- **Rviz中添加**
  - `/global_reflector_bars` (`visualization_msgs/MarkerArray`)：全局检测与优化反光柱可视化（黄色 + 蓝色圆柱 + id）。
  - `/reflector_markers` (`visualization_msgs/MarkerArray`)：徐工原始检测到的landmark（绿色圆柱）。



