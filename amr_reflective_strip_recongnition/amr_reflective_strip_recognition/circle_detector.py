import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.decomposition import PCA
from scipy.optimize import least_squares
import json
from sklearn.linear_model import RANSACRegressor
import os
from datetime import datetime
class ReflectorDetector(Node):
    """使用时间滤波结果的反光条检测节点"""
    
    def __init__(self):
        super().__init__('reflector_detector')
        
        # 订阅激光雷达数据
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # 发布检测到的反光条位姿（时间滤波后）
        self.publisher = self.create_publisher(
            PoseStamped, '/charger_relative_pose', 10)
        
        # 声明参数（保持不变）
        self.declare_parameter('intensity_threshold_use', 2000)
        self.declare_parameter('cluster_eps', 0.04)
        self.declare_parameter('min_cluster_points', 10)
        self.declare_parameter('diameter_min', 0.09)
        self.declare_parameter('diameter_max', 0.11)
        # self.declare_parameter('diameter_min', 0.12)
        # self.declare_parameter('diameter_max', 0.16)
        self.declare_parameter('residual_avg_threshold', 0.01)
        self.declare_parameter('residual_std_threshold', 0.005)
        self.declare_parameter('residual_max_threshold', 0.02)
        self.declare_parameter('stat_mean_k', 5)
        self.declare_parameter('stat_std_threshold', 1.0)
        self.declare_parameter('max_history_age', 3)   #10 maybe too long
        self.declare_parameter('match_distance_threshold', 0.1)
        self.declare_parameter('arc_threshold', 0.1)
        self.declare_parameter('max_arc_feature', 8.0)
        self.declare_parameter('arc_min_points', 8)
        self.declare_parameter('direction_tolerance', 0.785)
        # 新增：激光雷达相对于机器人基座的安装偏移参数（单位：米/弧度）
        self.declare_parameter('lidar_to_base_tx', 0.35)    # 激光雷达在基座x方向偏移（前为正）
        self.declare_parameter('lidar_to_base_ty', 0.0)    # 激光雷达在基座y方向偏移（左为正）
        self.declare_parameter('lidar_to_base_rz', 3.1416)    # 激光雷达在基座z轴旋转偏移（逆时针为正）
        
        # 读取参数值
        self.lidar_tx = self.get_parameter('lidar_to_base_tx').value
        self.lidar_ty = self.get_parameter('lidar_to_base_ty').value
        self.lidar_rz = self.get_parameter('lidar_to_base_rz').value
        
        self.declare_parameter('intensity_log_path', './intensity_logs')  # 新增：强度数据保存路径
        self.intensity_log_path = self.get_parameter('intensity_log_path').value
        os.makedirs(self.intensity_log_path, exist_ok=True)


        # 历史检测结果缓存
        self.detection_history = {}  # {id: (pose, diameter, confidence, age)}
        self.detection_id_counter = 0
        
        self.get_logger().info('使用时间滤波的反光条检测节点初始化完成')

    def getFinalCoor(self,points):
        final_x = -np.cos(points[2])*points[0]-np.sin(points[2])*points[1]
        final_y = np.sin(points[2])*points[0]-np.cos(points[2])*points[1]
        return [final_x,final_y,points[2]]

    def transform_to_reflector_frame(self, reflector_in_base_x, reflector_in_base_y, reflector_in_base_theta):
        """
        修正版：转换小车位姿到反光条坐标系（解决xy符号反的问题）
        输入：反光条在小车坐标系（base_link）中的位姿 (x, y, theta)
        输出：小车在反光条坐标系中的位姿 (x, y, theta)
        """
        # 关键修正：将旋转角度从 -reflector_in_base_theta 改为 reflector_in_base_theta
        # （原旋转方向与实际坐标系定义相反，导致符号颠倒）
        cos_theta = np.cos(reflector_in_base_theta)  # 修正：去掉负号
        sin_theta = np.sin(reflector_in_base_theta)  # 修正：去掉负号
        
        # 重新计算小车在反光条坐标系中的位置（基于修正后的旋转矩阵）
        # 公式含义：先将反光条在小车坐标系中的坐标 (rx, ry) 旋转，再取反
        car_in_reflector_x = (reflector_in_base_x * cos_theta - reflector_in_base_y * sin_theta)
        car_in_reflector_y = (reflector_in_base_x * sin_theta + reflector_in_base_y * cos_theta)  # 修正：sin/cos组合
        
        # 修正小车在反光条坐标系中的方向（保持与旋转方向一致）
        car_in_reflector_theta = -reflector_in_base_theta
        car_in_reflector_theta = (car_in_reflector_theta + np.pi) % (2 * np.pi) - np.pi  # 归一化到[-π, π]
        
        return car_in_reflector_x, car_in_reflector_y, car_in_reflector_theta
    

    def transform_to_base_link(self, lidar_x, lidar_y, lidar_theta):
        """
        二次坐标转换：激光雷达坐标系下的点→机器人基座坐标系（base_link）
        输入：lidar_x/lidar_y（激光雷达坐标系下的x/y，即car_position[0]/[1]）
            lidar_theta（激光雷达坐标系下的角度，即car_position[2]）
        输出：base_x/base_y/base_theta（基座坐标系下的x/y/theta）
        """
        # 1. 先对激光雷达的安装旋转偏移进行修正（绕基座原点旋转lidar_rz）
        # 旋转矩阵：绕z轴旋转lidar_rz角度
        cos_rz = np.cos(self.lidar_rz)
        sin_rz = np.sin(self.lidar_rz)
        
        # 旋转后的x/y（激光雷达相对于基座的旋转修正）
        rotated_x = lidar_x * cos_rz - lidar_y * sin_rz
        rotated_y = lidar_x * sin_rz + lidar_y * cos_rz
        
        # 2. 加上激光雷达相对于基座的平移偏移
        base_x = -rotated_x + self.lidar_tx
        base_y = -rotated_y + self.lidar_ty
        
        # 3. 角度修正（叠加激光雷达的旋转偏移）
        base_theta = lidar_theta + self.lidar_rz
        # 确保角度在[-π, π]范围内
        base_theta = (base_theta + np.pi) % (2 * np.pi) - np.pi
        
        return -base_x, -base_y, base_theta



    def statistical_outlier_filter(self, points):
        """统计离群点过滤"""
        if len(points) < 10:
            return points
            
        mean_k = self.get_parameter('stat_mean_k').value
        std_threshold = self.get_parameter('stat_std_threshold').value
        
        nbrs = NearestNeighbors(n_neighbors=mean_k, algorithm='ball_tree').fit(points)
        distances, _ = nbrs.kneighbors(points)
        
        mean_distances = np.mean(distances, axis=1)
        mean_dist = np.mean(mean_distances)
        std_dist = np.std(mean_distances)
        
        inlier_mask = mean_distances < mean_dist + std_threshold * std_dist
        return points[inlier_mask]

    def adaptive_dbscan(self, points):
        """自适应DBSCAN聚类"""
        if len(points) < self.get_parameter('min_cluster_points').value:
            return np.full(len(points), -1)
            
        k = 4
        nbrs = NearestNeighbors(n_neighbors=k).fit(points)
        distances, _ = nbrs.kneighbors(points)
        
        avg_distances = np.mean(np.sort(distances[:, 1:], axis=1), axis=1)
        median_eps = np.median(avg_distances)
        
        base_eps = self.get_parameter('cluster_eps').value
        eps = max(base_eps, median_eps)
        
        return DBSCAN(
            eps=eps,
            min_samples=self.get_parameter('min_cluster_points').value
        ).fit_predict(points)

    def circle_fit(self, points):
        """圆拟合及评估"""
        if len(points) < 3:
            return None, None, None, None, None

        x = points[:, 0]
        y = points[:, 1]

        def residuals(params):
            a, b, r = params
            return (x - a)**2 + (y - b)** 2 - r**2

        a0, b0 = np.mean(x), np.mean(y)
        r0 = np.mean(np.sqrt((x - a0)** 2 + (y - b0)**2))
        initial_guess = [a0, b0, r0]

        try:
            result = least_squares(residuals, initial_guess, method='lm')
            a, b, r = result.x
            
            residuals_values = residuals([a, b, r])
            avg_residual = np.mean(np.abs(residuals_values))
            std_residual = np.std(residuals_values)
            max_residual = np.max(np.abs(residuals_values))
            
            distances_to_center = np.sqrt((x - a)** 2 + (y - b)**2)
            ss_total = np.sum((distances_to_center - np.mean(distances_to_center))** 2)
            r_squared = 1 - np.sum(residuals_values**2) / ss_total if ss_total > 0 else 1.0
            
            return (a, b, r), avg_residual, std_residual, max_residual, r_squared
        except Exception as e:
            self.get_logger().debug(f"圆拟合失败: {e}")
            return None, None, None, None, None

    def get_primary_direction(self, points):
        """PCA计算主方向"""
        if len(points) < 2:
            return 0.0

        centered = points - np.mean(points, axis=0)
        pca = PCA(n_components=1)
        pca.fit(centered)
        direction = pca.components_[0]
        return np.arctan2(direction[1], direction[0])

    # def get_primary_direction(self, points):
    #     """改用RANSAC直线拟合估计主方向，适合少点、抗噪声"""
    #     if len(points) < 3:  # 至少3个点拟合直线
    #         return 0.0
        
    #     x = points[:, 0].reshape(-1, 1)  # RANSAC要求输入为2D数组
    #     y = points[:, 1]
        
    #     # 1. RANSAC直线拟合（残差阈值根据实际场景调整，单位：米）
    #     ransac = RANSACRegressor(
    #         residual_threshold=0.008,  # 允许的最大残差（点到直线的距离）
    #         min_samples=3,             # 拟合直线的最小点数
    #         max_trials=100             # 最大尝试次数，确保找到最优解
    #     )
    #     ransac.fit(x, y)
        
    #     # 2. 提取直线斜率，计算方向角
    #     # 直线方程：y = kx + b，方向向量为 (1, k)
    #     k = ransac.estimator_.coef_[0] if ransac.estimator_ is not None else 0.0
    #     direction_angle = np.arctan2(k, 1.0)  # 方向角：沿直线的水平向右方向
        
    #     return direction_angle

    def calculate_arc_feature(self, points):
        """计算弧度特征"""
        min_points = self.get_parameter('arc_min_points').value
        if len(points) < min_points:
            return 0.0
            
        primary_dir = self.get_primary_direction(points)
        center = np.mean(points, axis=0)
        rotated = np.dot(points - center, np.array([
            [np.cos(primary_dir), -np.sin(primary_dir)],
            [np.sin(primary_dir), np.cos(primary_dir)]
        ]))
        
        sorted_indices = np.argsort(rotated[:, 0])
        sorted_points = points[sorted_indices]
        
        n_points = len(sorted_points)
        curvature = np.zeros(n_points)
        window_size = min(5, n_points // 3)
        
        for i in range(window_size, n_points - window_size):
            window = sorted_points[i-window_size:i+window_size+1]
            circle, _, _, _, _ = self.circle_fit(window)
            if circle is not None:
                a, b, r = circle
                curvature[i] = 1.0 / r if r > 0 else 0.0
        
        valid_curvature = curvature[curvature > 0]
        if len(valid_curvature) < 3:
            return 0.0
            
        mean_curvature = np.mean(valid_curvature)
        std_curvature = np.std(valid_curvature)
        return mean_curvature / (std_curvature + 1e-6)

    def is_direction_towards_radar(self, direction_theta, center_x, center_y, tolerance=0.785):
        """判断方向是否朝向雷达"""
        towards_radar_theta = np.arctan2(-center_y, -center_x)
        angle_diff = np.abs(direction_theta - towards_radar_theta)
        angle_diff = np.min([angle_diff, 2 * np.pi - angle_diff])
        return angle_diff < tolerance

    def match_current_to_history(self, current_detections):
        """匹配当前检测与历史记录"""
        matches = {self._create_detection_hash(d): None for d in current_detections}
        threshold = self.get_parameter('match_distance_threshold').value
        
        hist_hashes = {
            self._create_detection_hash((h[0], h[1], h[2])): (k, h)
            for k, h in self.detection_history.items()
        }
        
        for detection in current_detections:
            pose, diameter, conf = detection
            current_hash = self._create_detection_hash(detection)
            current_pos = np.array([pose.pose.position.x, pose.pose.position.y])
            
            best_match = None
            min_distance = float('inf')
            for hist_hash, (hist_id, hist_data) in hist_hashes.items():
                hist_pose = hist_data[0]
                hist_pos = np.array([hist_pose.pose.position.x, hist_pose.pose.position.y])
                distance = np.linalg.norm(current_pos - hist_pos)
                if distance < min_distance and distance < threshold:
                    min_distance = distance
                    best_match = (hist_id, hist_data)
            
            if best_match:
                matches[current_hash] = best_match
                
        return matches

    def _create_detection_hash(self, detection):
        """创建检测结果哈希"""
        pose, diameter, confidence = detection
        pos = pose.pose.position
        ori = pose.pose.orientation
        
        return (
            round(pos.x, 4),
            round(pos.y, 4),
            round(pos.z, 4),
            round(ori.x, 4),
            round(ori.y, 4),
            round(ori.z, 4),
            round(ori.w, 4),
            round(diameter, 4)
        )

    def fuse_poses(self, pose1, pose2, weight1, weight2):
        """融合两个位姿"""
        fused_pose = PoseStamped()
        total_weight = weight1 + weight2
        
        fused_pose.pose.position.x = (pose1.pose.position.x * weight1 + pose2.pose.position.x * weight2) / total_weight
        fused_pose.pose.position.y = (pose1.pose.position.y * weight1 + pose2.pose.position.y * weight2) / total_weight
        fused_pose.pose.position.z = 0.0
        
        q1 = np.array([pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w])
        q2 = np.array([pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z, pose2.pose.orientation.w])
        if np.dot(q1, q2) < 0:
            q2 = -q2
            
        t = weight1 / total_weight
        q_fused = (1 - t) * q2 + t * q1
        q_fused /= np.linalg.norm(q_fused)
        
        fused_pose.pose.orientation.x = q_fused[0]
        fused_pose.pose.orientation.y = q_fused[1]
        fused_pose.pose.orientation.z = q_fused[2]
        fused_pose.pose.orientation.w = q_fused[3]
        
        return fused_pose



    def save_intensities_to_json(self, intensities, msg):
        """将强度数据保存到JSON文件"""
        # 创建保存数据的字典
        data = {
            "timestamp": datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9).isoformat(),
            "frame_id": msg.header.frame_id,
            "intensity_threshold": self.get_parameter('intensity_threshold_use').value,
            "num_points": len(intensities),
            "intensities": intensities.tolist()  # 转换为Python列表以支持JSON序列化
        }
        
        # 生成唯一的文件名（基于时间戳）
        filename = f"intensities_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.json"
        file_path = os.path.join(self.intensity_log_path, filename)
        
        # 保存到JSON文件
        try:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().debug(f"强度数据已保存到: {file_path}")
        except Exception as e:
            self.get_logger().error(f"保存强度数据失败: {e}")


    def update_temporal_filter(self, current_detections):
        """时间滤波：融合多帧结果"""
        matches = self.match_current_to_history(current_detections)
        max_age = self.get_parameter('max_history_age').value
        detection_hash_map = {self._create_detection_hash(d): d for d in current_detections}
        filtered_detections = []
        
        for detection_hash, match_info in matches.items():
            detection = detection_hash_map[detection_hash]
            pose, diameter, confidence = detection
            
            if match_info is not None:
                # 匹配到历史记录，融合结果
                hist_id, hist_data = match_info
                hist_pose, hist_diameter, hist_conf, hist_age = hist_data
                fused_pose = self.fuse_poses(pose, hist_pose, confidence, hist_conf)
                fused_confidence = (confidence + hist_conf * 0.8) / 1.8
                self.detection_history[hist_id] = (fused_pose, diameter, fused_confidence, 0)
                filtered_detections.append((fused_pose, diameter, fused_confidence))
            else:
                # 新检测结果，添加到历史
                new_id = self.detection_id_counter
                self.detection_id_counter += 1
                self.detection_history[new_id] = (pose, diameter, confidence, 0)
                filtered_detections.append((pose, diameter, confidence))
        
        # 老化未匹配的历史记录
        for hist_id in list(self.detection_history.keys()):
            hist_pose, hist_diameter, hist_conf, hist_age = self.detection_history[hist_id]
            if hist_age > max_age:
                del self.detection_history[hist_id]
            else:
                self.detection_history[hist_id] = (
                    hist_pose, hist_diameter, hist_conf * 0.95, hist_age + 1
                )
        
        return filtered_detections

    def scan_callback(self, msg):
        """激光雷达数据回调：处理并发布时间滤波后的结果"""
        # 1. 提取高强度点
        intensity_thresh = self.get_parameter('intensity_threshold_use').value
        # self.get_logger().info(f"实际使用的强度阈值: {intensity_thresh}")
    
        # 转换为NumPy数组（一次性处理所有点）
        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities)

        # self.save_intensities_to_json(intensities, msg)
        
        # 计算所有点的角度（使用angle_increment保证精度）
        num_points = len(ranges)
        angles = msg.angle_min + np.arange(num_points) * msg.angle_increment  # 替代linspace，精度更高
        
        valid_mask1 = (intensities > intensity_thresh)
        # self.get_logger().info(f"通过强度筛选的点数量: {np.sum(valid_mask1)}")

        # 筛选有效点（向量化条件判断）
        valid_mask = (
            (intensities > intensity_thresh) &  # 强度阈值
            (ranges > msg.range_min) &          # 最小距离
            (ranges < msg.range_max) &          # 最大距离
            (~np.isnan(ranges))                 # 非NaN值
        )
        
        # 提取有效点并计算坐标（向量化运算）
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        points = np.column_stack([
            valid_ranges * np.cos(valid_angles),  # x坐标
            valid_ranges * np.sin(valid_angles)   # y坐标
        ])
        
        if len(points) == 0:
            # 无点时发布空姿态
            # empty_pose = PoseStamped(header=msg.header)
            # self.publisher.publish(empty_pose)
            self.get_logger().debug("未检测到高强度点，不发布结果")
            return
        
        # 2. 去噪
        if len(points) > 10:
            points = self.statistical_outlier_filter(points)
        
        # 3. 聚类
        if len(points) >= self.get_parameter('min_cluster_points').value:
            labels = self.adaptive_dbscan(points)
        else:
            labels = np.full(len(points), -1)
        
        valid_points = points[labels != -1]
        valid_labels = labels[labels != -1]
        if len(valid_points) == 0:
            # empty_pose = PoseStamped(header=msg.header)
            # self.publisher.publish(empty_pose)
            self.get_logger().debug("未检测到有效聚类，不发布结果")
            return
        
        # 4. 圆拟合与验证（收集原始检测结果）
        current_detections = []
        unique_labels = np.unique(valid_labels)
        direction_tolerance = self.get_parameter('direction_tolerance').value
        
        for label in unique_labels:
            cluster = valid_points[valid_labels == label]
            circle, avg_residual, std_residual, max_residual, r_squared = self.circle_fit(cluster)
            
            if circle is None or r_squared < 0.85:
                continue
            
            a, b, r = circle
            diameter = 2 * r
            diameter_min, diameter_max = self.get_parameter('diameter_min').value, self.get_parameter('diameter_max').value
            if not (diameter_min <= diameter <= diameter_max):
                continue
            
            arc_feature = self.calculate_arc_feature(cluster)
            arc_threshold, max_arc = self.get_parameter('arc_threshold').value, self.get_parameter('max_arc_feature').value
            if (avg_residual > self.get_parameter('residual_avg_threshold').value or
                std_residual > self.get_parameter('residual_std_threshold').value or
                max_residual > self.get_parameter('residual_max_threshold').value or
                arc_feature < arc_threshold or arc_feature > max_arc):
                continue
            
            # 计算中心和方向
            arc_center = np.mean(cluster, axis=0)
            center_x, center_y = arc_center[0], arc_center[1]
            tangent_theta = self.get_primary_direction(cluster)
            normal_theta = tangent_theta - np.pi/2
            
            # 方向修正
            if not self.is_direction_towards_radar(normal_theta, center_x, center_y, direction_tolerance):
                normal_theta += np.pi
                normal_theta = (normal_theta + np.pi) % (2 * np.pi) - np.pi
                if not self.is_direction_towards_radar(normal_theta, center_x, center_y, direction_tolerance):
                    continue

            car_position = self.getFinalCoor([center_x,center_y,normal_theta])
            reflector_in_base_x, reflector_in_base_y, reflector_in_base_theta = self.transform_to_base_link(
                car_position[0], car_position[1], car_position[2]
                )

            # 转换为“小车在反光条坐标系中的位姿”
            car_rx, car_ry, car_rtheta = self.transform_to_reflector_frame(
                reflector_in_base_x,  # 反光条在基座的x
                reflector_in_base_y,  # 反光条在基座的y
                reflector_in_base_theta  # 反光条在基座的方向
            )
            
            # 构造位姿（若需要发布小车在反光条坐标系的位姿，可基于car_rx/car_ry构造）
            pose = PoseStamped()
            pose.header = msg.header
            # pose.header.frame_id = "reflector_frame"  # 标注为反光条坐标系
            pose.pose.position.x = car_rx  # 小车在反光条坐标系的x
            pose.pose.position.y = car_ry  # 小车在反光条坐标系的y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = np.sin(car_rtheta / 2)  # 小车在反光条坐标系的方向
            pose.pose.orientation.w = np.cos(car_rtheta / 2)
            
            # # 构造位姿：使用二次转换后的坐标（base_x/base_y）和修正后的角度（base_theta）
            # pose = PoseStamped()
            # pose.header = msg.header
            # # pose.header.frame_id = "base_link"  # 关键：更新坐标系为基座坐标系（避免混淆）
            # pose.pose.position.x = base_x       # 二次转换后的x
            # pose.pose.position.y = base_y       # 二次转换后的y
            # pose.pose.position.z = 0.0
            # # 角度使用二次转换后的base_theta
            # pose.pose.orientation.z = np.sin(base_theta / 2)
            # pose.pose.orientation.w = np.cos(base_theta / 2)
            
            # 计算置信度并添加到原始检测列表
            confidence = min(1.0, 0.8 + 0.2 * (len(cluster) / 20)) * min(1.0, r_squared) * min(1.0, arc_feature / arc_threshold)
            current_detections.append((pose, diameter, confidence))
        
        # 5. 应用时间滤波并发布结果
        if current_detections:
            # 对原始检测结果进行时间滤波
            filtered_detections = self.update_temporal_filter(current_detections)
            
            # 从滤波结果中选择置信度最高的位姿发布
            filtered_detections.sort(key=lambda x: x[2], reverse=True)  # 按置信度降序排序
            best_pose = filtered_detections[0][0]  # 取最优结果
            best_pose.header = msg.header  # 更新时间戳
            self.publisher.publish(best_pose)
            
            # 打印滤波后结果日志
            theta = 2 * np.arctan2(best_pose.pose.orientation.z, best_pose.pose.orientation.w)
            self.get_logger().info(
                f"发布时间滤波后结果：中心({best_pose.pose.position.x:.3f}m, {best_pose.pose.position.y:.3f}m), "
                f"方向={theta:.3f}rad, 置信度={filtered_detections[0][2]:.2f}"
            )
        else:
            # 无有效检测时发布空姿态
            # empty_pose = PoseStamped(header=msg.header)
            # self.publisher.publish(empty_pose)
            self.get_logger().debug("未检测到有效反光条，不发布结果")

def main(args=None):
    rclpy.init(args=args)
    node = ReflectorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
