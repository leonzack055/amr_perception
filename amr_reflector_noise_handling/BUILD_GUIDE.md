# 构建和测试指南

## 1. 构建包

在工作空间根目录下执行：

```bash
cd /workspaces/ros-dev/amr_ws

# 清理旧的构建（如果需要）
rm -rf build install log

# 构建整个工作空间
colcon build --packages-select amr_reflector_noise_handling

# 或者使用symlink安装以加快开发速度
colcon build --packages-select amr_reflector_noise_handling --symlink-install
```

## 2. 检查构建结果

```bash
# 检查可执行文件是否生成
ls -la install/amr_reflector_noise_handling/lib/amr_reflector_noise_handling/

# 应该看到：
# reflector_noise_node
```

## 3. 运行节点

### 基本运行

```bash
# Source工作空间
source install/setup.bash

# 运行节点（默认参数）
ros2 run amr_reflector_noise_handling reflector_noise_node

# 使用自定义参数
ros2 run amr_reflector_noise_handling reflector_noise_node --ros-args -p intensity_threshold_base:=30.0
```

### 使用launch文件

```bash
# 使用默认配置
ros2 launch amr_reflector_noise_handling reflector_noise_handling.launch.py

# 使用自定义配置文件
ros2 launch amr_reflector_noise_handling reflector_noise_handling.launch.py params_file:=/path/to/custom_params.yaml

# 自定义话题名称
ros2 launch amr_reflector_noise_handling reflector_noise_handling.launch.py scan_topic:=/custom_scan landmark_topic:=/custom_landmarks
```

## 4. 测试检测效果

### 查看输出话题

```bash
# 查看检测到的地标
ros2 topic echo /landmark_noise

# 查看可视化标记
ros2 topic echo /reflector_noise_markers

# 查看节点参数
ros2 param list /reflector_noise_handling

# 查看特定参数
ros2 param get /reflector_noise_handling intensity_threshold_base
```

### 使用RViz可视化

```bash
ros2 run rviz2 rviz2
```

在RViz中添加以下显示：
- **LaserScan**: `/scan` - 原始激光数据
- **MarkerArray**: `/reflector_noise_markers` - 检测到的反光柱标记
- **PoseArray**: `/landmark_noise` (如果需要) - 地标位姿

## 5. 参数调优

### 动态调整参数

```bash
# 设置强度阈值
ros2 param set /reflector_noise_handling intensity_threshold_base 40.0

# 设置预期直径
ros2 param set /reflector_noise_handling expected_diameter 0.05

# 设置直径容差
ros2 param set /reflector_noise_handling diameter_tolerance 0.03

# 设置最小置信度
ros2 param set /reflector_noise_handling min_confidence 0.6
```

### 保存参数到文件

```bash
# 导出当前参数
ros2 param dump /reflector_noise_handling

# 编辑参数文件后重新加载
ros2 param load /reflector_noise_handling /path/to/params.yaml
```

## 6. 性能测试

### 测试不同距离下的检测效果

1. **近距离测试 (0.5-2m)**
   - 验证：点数多时能否正确聚类
   - 关注：误检率

2. **中距离测试 (2-5m)**
   - 验证：标准情况下的检测精度
   - 关注：定位精度

3. **远距离测试 (5-8m)**
   - 验证：点数少时的检测稳定性
   - 关注：漏检率

### 录制和分析数据

```bash
# 录制数据包
ros2 bag record -o test_reflector_detection /scan /landmark_noise /reflector_noise_markers

# 回放数据包
ros2 bag play test_reflector_detection

# 分析数据包
ros2 bag info test_reflector_detection
```

## 7. 调试技巧

### 启用详细日志

```bash
# 设置日志级别为DEBUG
ros2 run amr_reflector_noise_handling reflector_noise_node --ros-args --log-level debug

# 或者针对特定模块
ros2 run amr_reflector_noise_handling reflector_noise_node --ros-args --log-level reflector_noise_handling:=debug
```

### 查看节点统计信息

```bash
# 查看节点频率
ros2 topic hz /landmark_noise

# 查看节点延迟
ros2 topic delay /landmark_noise

# 查看节点带宽
ros2 topic bw /landmark_noise
```

### 检查TF变换

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo base_link laser
```

## 8. 常见问题排查

### 问题1：编译错误

```bash
# 清理并重新构建
cd /workspaces/ros-dev/amr_ws
rm -rf build install log
colcon build --packages-select amr_reflector_noise_handling --cmake-clean-cache
```

### 问题2：运行时找不到节点

```bash
# 确认source了工作空间
source install/setup.bash

# 检查包是否正确安装
ros2 pkg prefix amr_reflector_noise_handling
```

### 问题3：没有检测到反光柱

检查：
1. 激光数据是否正常：`ros2 topic echo /scan`
2. 强度阈值是否过高：降低 `intensity_threshold_base`
3. 直径容差是否过严：增加 `diameter_tolerance`
4. 最小置信度是否过高：降低 `min_confidence`

### 问题4：误检太多

检查：
1. 强度阈值是否过低：提高 `intensity_threshold_base`
2. 最小点数是否过少：增加 `min_points_coarse`
3. 几何验证是否开启：确保 `enable_geometric_validation=true`

## 9. 与Cartographer集成

### 配置Cartographer使用检测到的地标

在Cartographer配置文件中添加：

```lua
-- 使用地标进行约束
TRAJECTORY_BUILDER.tracker_options.use_landmarks = true

-- 地标置信度阈值
TRAJECTORY_BUILDER.tracker_options.landmark_confidence_threshold = 0.7

-- 地标匹配距离阈值
TRAJECTORY_BUILDER.tracker_options.landmark_match_distance_threshold = 0.1
```

### 启动完整SLAM系统

```bash
# 启动反光柱检测
ros2 launch amr_reflector_noise_handling reflector_noise_handling.launch.py

# 启动Cartographer SLAM
ros2 launch cartographer_ros your_cartographer_launch_file.lua
```

## 10. 性能基准测试

### 预期性能指标

| 指标 | 近距离(<2m) | 中距离(2-5m) | 远距离(>5m) |
|------|------------|-------------|------------|
| 检测率 | >95% | >98% | >90% |
| 误检率 | <5% | <2% | <8% |
| 定位误差 | <2cm | <3cm | <5cm |
| 处理延迟 | <20ms | <15ms | <10ms |

### 性能测试脚本

```bash
#!/bin/bash
# performance_test.sh

echo "Starting performance test..."

# 测试检测率
ros2 topic hz /landmark_noise &
HZ_PID=$!

# 运行测试时间（秒）
TEST_DURATION=60

echo "Running for $TEST_DURATION seconds..."
sleep $TEST_DURATION

# 停止测试
kill $HZ_PID

echo "Performance test completed."
```

## 11. 下一步开发

### 优化方向

1. **深度学习集成**：使用训练好的模型辅助检测
2. **多帧融合**：结合多帧数据提高稳定性
3. **自适应参数**：根据环境自动调整参数
4. **GPU加速**：使用CUDA加速聚类算法

### 扩展功能

1. **反光柱ID识别**：通过二维码或颜色编码识别不同反光柱
2. **动态更新地图**：在线更新反光柱位置
3. **异常检测**：检测反光柱被遮挡或移动的情况