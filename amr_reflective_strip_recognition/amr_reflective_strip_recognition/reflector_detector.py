
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray,PoseStamped
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor,LinearRegression
class ReflectorDetector(Node):
    def __init__(self):
        super().__init__('reflector_detector')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)
        self.publisher = self.create_publisher(
            PoseArray, '/reflectors', 10)
        
        # 参数声明
        self.declare_parameter('intensity_threshold', 1000)
        self.declare_parameter('min_points', 5)
        self.declare_parameter('reflector_width', 0.05)

    def get_line_width(self,data_,ransac_):
        k = ransac_.estimator_.coef_[0]
        b = ransac_.estimator_.intercept_
        if np.isinf(k):
            x_min,x_max = b,b
            y_min,y_max = data_[:,1].min(),data_[:,1].max()
        else:
            x_min,x_max = data_[:,0].min(),data_[:,0].max()
            y_min,y_max = k*x_min +b,k*x_max +b
        start_point = np.array([x_min,y_min])
        end_point = np.array([x_max,y_max])
        vector_ = end_point - start_point
        theta_rad = np.arctan2(vector_[1],vector_[0])+np.pi/2
        segment_length = np.linalg.norm(end_point-start_point)
        return segment_length,theta_rad

    def callback(self, msg):
        # 1.强度
        points = np.array([
            [r*np.cos(θ), r*np.sin(θ)]
            for i, (r, θ) in enumerate(zip(
                msg.ranges,
                np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            )) if msg.intensities[i] > self.get_parameter('intensity_threshold').value
            and msg.range_min < r < msg.range_max
        ])
        # DBSCAN聚类^^2^^3^^
        if len(points) > 2:
            labels = DBSCAN(eps=0.05, min_samples=2).fit_predict(points)
        else:
            labels = np.full(len(points),-1)

        valid_clusters = points[labels != -1]
        valid_labels = labels[labels != -1]

        # RANSAC直线拟合^^4^^5^^
        results = PoseArray()
        results.header = msg.header
        unique_labels = np.unique(valid_labels)
        for label in unique_labels:
            cluster_points = valid_clusters[valid_labels == label]
            X = cluster_points[:, 0].reshape(-1, 1)
            Y = cluster_points[:, 1]
            
            ransac = RANSACRegressor(
                estimator=LinearRegression(),
                min_samples=2,
                residual_threshold=0.01,
                max_trials = 100
            )
            ransac.fit(X, Y)
            inlier_mask = ransac.inlier_mask_
            if inlier_mask is not None:
                inliers = cluster_points[inlier_mask]
                line_w,angle_line = self.get_line_width(inliers,ransac)
                degree_data = np.degrees(angle_line)
                if line_w > self.get_parameter('reflector_width').value-0.01 and line_w < self.get_parameter('reflector_width').value+0.01:
                    midpoint_ = np.mean(inliers,axis=0)
                    #line_x = np.array([cluster_points[:0].min(),cluster_points[:0].max()]).reshape(-1,1)
                    #line_y = ransac.predict(line_x)
                    #midpoint_ = np.array([line_x.mean(),line_y.mean()])    
                    pose = PoseStamped()
                    pose.pose.position.x = midpoint_[0]
                    pose.pose.position.y = midpoint_[1]
                    pose.pose.orientation.z = np.sin(angle_line/2)
                    pose.pose.orientation.w = np.cos(angle_line/2)
                    self.get_logger().info(f"Current position: x={midpoint_[0]:.3f}, y={midpoint_[1]:.3f}, w={line_w:.3f},angle={degree_data:.3f}")
                    results.poses.append(pose.pose)
        self.publisher.publish(results)
def main(args=None):
    rclpy.init(args=args)
    node = ReflectorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
