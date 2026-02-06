#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cartographer_ros_msgs/msg/landmark_list.hpp>
#include <cartographer_ros_msgs/msg/landmark_entry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/fixed_dbscan.hpp"
#include "amr_reflector_noise_handling/fractal_dimension.hpp"
#include "amr_reflector_noise_handling/pca_shape_classifier.hpp"
#include "amr_reflector_noise_handling/improved_interpolation.hpp"
#include "amr_reflector_noise_handling/circle_fitter.hpp"
#include "amr_reflector_noise_handling/practical_descriptor.hpp"
#include "amr_reflector_noise_handling/geometric_validator.hpp"
#include "amr_reflector_noise_handling/reflector_tracker.hpp"

using namespace amr_reflector_noise_handling;

class ReflectorNoiseHandlingNode : public rclcpp::Node {
public:
    ReflectorNoiseHandlingNode()
        : Node("reflector_noise_handling_node")
    {
        // Declare parameters
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("landmark_topic", "/landmark_noise");
        this->declare_parameter("marker_topic", "/reflector_noise_markers");
        this->declare_parameter("compensated_cloud_topic", "/compensated_cloud");
        this->declare_parameter("classification_method", "fractal_dimension");  // "fractal_dimension" or "pca"
        this->declare_parameter("raw_intensity_threshold", 1000.0);  // Fixed raw threshold
        this->declare_parameter("expected_diameter", 0.07);  // 7cm diameter
        this->declare_parameter("diameter_tolerance", 0.03);
        this->declare_parameter("enable_interpolation", true);
        this->declare_parameter("enable_tracking", true);
        this->declare_parameter("min_confidence", 0.5);
        this->declare_parameter("tracking_time_gap", 2.0);  // 2 seconds
        this->declare_parameter("match_threshold", 0.8);  // Similarity threshold
        
        // Fractal dimension parameters
        this->declare_parameter("fractal_dimension.enable", true);
        this->declare_parameter("fractal_dimension.min_fd_for_post", 1.0);
        this->declare_parameter("fractal_dimension.max_fd_for_post", 1.3);
        this->declare_parameter("fractal_dimension.min_density", 50.0);
        
        // PCA classification parameters
        this->declare_parameter("pca_classification.enable", true);
        this->declare_parameter("pca_classification.max_elongation_post", 1.5);
        this->declare_parameter("pca_classification.min_elongation_board", 2.0);
        this->declare_parameter("pca_classification.max_linearity_post", 0.6);
        this->declare_parameter("pca_classification.min_linearity_board", 0.8);
        this->declare_parameter("pca_classification.min_circularity_post", 0.7);
        this->declare_parameter("pca_classification.max_circularity_board", 0.5);
        this->declare_parameter("pca_classification.min_points", 5);
        
        // Improved interpolation parameters
        this->declare_parameter("interpolation.min_points", 5);
        this->declare_parameter("interpolation.max_points", 20);
        this->declare_parameter("interpolation.min_distance", 2.0);
        this->declare_parameter("interpolation.max_distance", 4.0);
        this->declare_parameter("interpolation.gap_multiplier", 1.5);
        
        // Circle fit parameters
        this->declare_parameter("circle_fit.max_fit_error", 0.03);
        this->declare_parameter("circle_fit.min_inlier_ratio", 0.5);
        this->declare_parameter("circle_fit.max_fit_error_near", 0.04);
        this->declare_parameter("circle_fit.max_fit_error_far", 0.02);
        this->declare_parameter("circle_fit.far_distance_threshold", 3.0);
        
        // Get parameters
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string landmark_topic = this->get_parameter("landmark_topic").as_string();
        std::string marker_topic = this->get_parameter("marker_topic").as_string();
        std::string compensated_cloud_topic = this->get_parameter("compensated_cloud_topic").as_string();
        classification_method_ = this->get_parameter("classification_method").as_string();
        double expected_diameter = this->get_parameter("expected_diameter").as_double();
        double diameter_tolerance = this->get_parameter("diameter_tolerance").as_double();
        raw_intensity_threshold_ = this->get_parameter("raw_intensity_threshold").as_double();
        enable_interpolation_ = this->get_parameter("enable_interpolation").as_bool();
        enable_tracking_ = this->get_parameter("enable_tracking").as_bool();
        min_confidence_ = this->get_parameter("min_confidence").as_double();
        
        // Configure fractal dimension calculator
        ClassificationParams fd_params;
        fd_params.min_fd_for_post = this->get_parameter("fractal_dimension.min_fd_for_post").as_double();
        fd_params.max_fd_for_post = this->get_parameter("fractal_dimension.max_fd_for_post").as_double();
        fd_params.min_density = this->get_parameter("fractal_dimension.min_density").as_double();
        
        // Configure PCA classifier
        ShapeClassificationParams pca_params;
        pca_params.max_elongation_post = this->get_parameter("pca_classification.max_elongation_post").as_double();
        pca_params.min_elongation_board = this->get_parameter("pca_classification.min_elongation_board").as_double();
        pca_params.min_linearity_board = this->get_parameter("pca_classification.min_linearity_board").as_double();
        pca_params.max_linearity_post = this->get_parameter("pca_classification.max_linearity_post").as_double();
        pca_classifier_.setParams(pca_params);
        
        // Configure improved interpolator
        ImprovedInterpolationCompensator::InterpolationParams interp_params;
        interp_params.min_points = this->get_parameter("interpolation.min_points").as_int();
        interp_params.max_points = this->get_parameter("interpolation.max_points").as_int();
        interp_params.min_distance = this->get_parameter("interpolation.min_distance").as_double();
        interp_params.max_distance = this->get_parameter("interpolation.max_distance").as_double();
        interp_params.gap_multiplier = this->get_parameter("interpolation.gap_multiplier").as_double();
        improved_interpolator_.setParams(interp_params);
        
        // Configure circle fitter
        CircleFitParams circle_params;
        circle_params.max_fit_error = this->get_parameter("circle_fit.max_fit_error").as_double();
        circle_params.min_inlier_ratio = this->get_parameter("circle_fit.min_inlier_ratio").as_double();
        circle_params.max_fit_error_near = this->get_parameter("circle_fit.max_fit_error_near").as_double();
        circle_params.max_fit_error_far = this->get_parameter("circle_fit.max_fit_error_far").as_double();
        circle_params.far_distance_threshold = this->get_parameter("circle_fit.far_distance_threshold").as_double();
        circle_fitter_.setParams(circle_params);
        
        // Configure geometric validator
        geometric_validator_.setExpectedDiameter(expected_diameter);
        geometric_validator_.setDiameterTolerance(diameter_tolerance);
        
        // Configure tracker
        tracker_.setNode(this);
        tracker_.setMaxTimeGap(this->get_parameter("tracking_time_gap").as_double());
        tracker_.setMatchThreshold(this->get_parameter("match_threshold").as_double());
        
        // Create subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&ReflectorNoiseHandlingNode::scan_callback, this, std::placeholders::_1));
        
        // Create publishers
        landmark_pub_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>(
            landmark_topic, 10);
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic, 10);
        
        // Debug info publisher
        debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reflector_debug_markers", 10);
        
        // Compensated point cloud publisher for visualization
        compensated_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            compensated_cloud_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "反光柱噪声处理节点初始化完成 (分形维数+改进插值+圆拟合+PCA分类版本)");
        RCLCPP_INFO(this->get_logger(), "分类方法: %s", classification_method_.c_str());
        RCLCPP_INFO(this->get_logger(), "扫描话题: %s", scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "地标话题: %s", landmark_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "标记话题: %s", marker_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "补偿点云话题: %s", compensated_cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "原始强度阈值: %.0f", raw_intensity_threshold_);
        RCLCPP_INFO(this->get_logger(), "期望直径: %.3f m", expected_diameter);
        RCLCPP_INFO(this->get_logger(), "直径容差: %.3f m", diameter_tolerance);
        RCLCPP_INFO(this->get_logger(), "插值补偿: %s", enable_interpolation_ ? "启用" : "禁用");
        RCLCPP_INFO(this->get_logger(), "追踪系统: %s", enable_tracking_ ? "启用" : "禁用");
        RCLCPP_INFO(this->get_logger(), "最小置信度: %.2f", min_confidence_);
        RCLCPP_INFO(this->get_logger(), "追踪时间窗口: %.1f s", tracker_.getMaxTimeGap());
        RCLCPP_INFO(this->get_logger(), "匹配相似度阈值: %.2f", tracker_.getMatchThreshold());
        if (classification_method_ == "fractal_dimension") {
            RCLCPP_INFO(this->get_logger(), "分形维数过滤: 启用 (FD范围: [%.1f, %.1f])",
                        fd_params.min_fd_for_post, fd_params.max_fd_for_post);
        } else if (classification_method_ == "pca") {
            RCLCPP_INFO(this->get_logger(), "PCA形状分类: 启用 (延伸度阈值: [%.1f, %.1f], 线性度: [%.1f, %.1f])",
                        pca_params.max_elongation_post, pca_params.min_elongation_board,
                        pca_params.max_linearity_post, pca_params.min_linearity_board);
        }
        RCLCPP_INFO(this->get_logger(), "改进插值: 启用 (距离范围: [%.1f, %.1f]m)",
                    interp_params.min_distance, interp_params.max_distance);
        RCLCPP_INFO(this->get_logger(), "圆拟合验证: 启用 (最大误差: %.3fm)", circle_params.max_fit_error);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // Phase 1: Convert laser scan to points
        std::vector<Point> points = convertScanToPoints(scan_msg);
        
        if (points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "扫描数据为空");
            return;
        }
        
        // Phase 1: Apply fixed intensity threshold (1000)
        auto filtered_points = filterByIntensity(points, raw_intensity_threshold_);
        
        if (filtered_points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "强度过滤后无点");
            return;
        }

        // Publish processed point cloud
        publishCompensatedCloud(filtered_points, scan_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "原始点数: %zu, 强度过滤后: %zu", 
                    points.size(), filtered_points.size());
        
        // Phase 1: Fixed DBSCAN clustering (EPS=10cm, min_points=5)
        auto cluster_indices = fixed_dbscan_.cluster(filtered_points);
        
        if (cluster_indices.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "DBSCAN聚类后无簇");
            return;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "DBSCAN聚类得到 %zu 个簇", cluster_indices.size());
        
        // Convert indices to actual point clusters
        std::vector<std::vector<Point>> clusters;
        for (const auto& indices : cluster_indices) {
            std::vector<Point> cluster;
            cluster.reserve(indices.size());
            for (int idx : indices) {
                cluster.push_back(filtered_points[idx]);
            }
            clusters.push_back(cluster);
        }
        
        // Phase 2: Apply classification and improved processing
        std::vector<DetectedReflector> reflectors;
        
        for (size_t idx = 0; idx < clusters.size(); ++idx) {
            const auto& cluster = clusters[idx];
            
            // Compute cluster center
            Point center = computeCentroid(cluster);
            double distance = center.distanceFromOrigin();
            
            RCLCPP_DEBUG(this->get_logger(), "簇 %zu: %zu 点, 距离 %.2fm",
                        idx, cluster.size(), distance);
            
            // Step 1: Classification (Fractal Dimension or PCA)
            bool is_reflector_candidate = false;
            std::string cluster_type_str;
            FractalDimensionResult fd_result;
            ShapeFeatures pca_features;
            double coverage = 0.0;  // For PCA method
            
            if (classification_method_ == "fractal_dimension") {
                // Fractal dimension classification
                ClassificationParams fd_params;
                fd_params.min_fd_for_post = this->get_parameter("fractal_dimension.min_fd_for_post").as_double();
                fd_params.max_fd_for_post = this->get_parameter("fractal_dimension.max_fd_for_post").as_double();
                fd_params.min_density = this->get_parameter("fractal_dimension.min_density").as_double();
                
                // Compute fractal dimension result
                fd_result = fd_calculator_.computeSimplified(cluster, center);
                double density = fd_calculator_.computeDensity(cluster, center);
                auto [coverage, max_gap] = fd_calculator_.computeAngularCoverage(cluster, center);
                
                // Classify cluster
                auto cluster_type = fd_calculator_.classifyCluster(cluster, center, fd_params);
                
                // Log cluster information with fractal dimension
                switch (cluster_type) {
                    case REFLECTOR_POST_CANDIDATE:
                        cluster_type_str = "反光柱候选";
                        is_reflector_candidate = true;
                        break;
                    case REFLECTOR_BOARD_CANDIDATE:
                        cluster_type_str = "反光板候选";
                        break;
                    case NOISE_CLUSTER:
                        cluster_type_str = "噪声簇";
                        break;
                    default:
                        cluster_type_str = "未知";
                        break;
                }
                
                RCLCPP_INFO(this->get_logger(),
                           "簇 %zu: 点数=%zu, 距离=%.2fm, FD=%.3f, CV=%.3f, 密度=%.1f, 覆盖率=%.2fπ, 类型=%s",
                           idx, cluster.size(), distance, fd_result.dimension, fd_result.cv,
                           density, coverage / M_PI, cluster_type_str.c_str());
                
            } else if (classification_method_ == "pca") {
                // PCA shape classification
                pca_features = pca_classifier_.computeShapeFeatures(cluster);
                auto object_type = pca_classifier_.classifyObject(pca_features);
                
                // Compute coverage for descriptor
                coverage = pca_classifier_.computeAngularCoverage(cluster, pca_features.center);
                
                // Log cluster information with PCA features
                switch (object_type) {
                    case REFLECTOR_POST:
                        cluster_type_str = "反光柱";
                        is_reflector_candidate = true;
                        break;
                    case REFLECTOR_BOARD:
                        cluster_type_str = "反光板";
                        break;
                    case CLUSTER_NOISE:
                        cluster_type_str = "噪声簇";
                        break;
                    case OBJECT_UNKNOWN:
                    default:
                        cluster_type_str = "未知";
                        break;
                }
                
                RCLCPP_INFO(this->get_logger(),
                           "簇 %zu: 点数=%zu, 距离=%.2fm, 延伸度=%.2f, 线性度=%.2f, 圆形度=%.2f, 类型=%s",
                           idx, cluster.size(), distance, pca_features.elongation,
                           pca_features.linearity, pca_features.circularity, cluster_type_str.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "未知的分类方法: %s", classification_method_.c_str());
                continue;
            }
            
            if (!is_reflector_candidate) {
                RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 非反光柱候选，跳过", idx);
                continue;  // Skip non-reflector candidates
            }
            
            RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 通过分类过滤", idx);
            
            // Step 2: Improved interpolation (if needed)
            auto processed_cluster = cluster;
            if (enable_interpolation_) {
                processed_cluster = improved_interpolator_.interpolateCluster(cluster, center, distance);
                
                if (processed_cluster.size() > cluster.size()) {
                    RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 插值补偿 %zu -> %zu 点",
                                idx, cluster.size(), processed_cluster.size());
                }
            }
            
            // Step 3: Circle fitting
            auto circle_fit = circle_fitter_.fitCircle(processed_cluster);
            
            RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 圆拟合 - 半径=%.3fm, 误差=%.3fm, 内点比例=%.2f",
                        idx, circle_fit.radius, circle_fit.fit_error, circle_fit.inlier_ratio);
            
            // Step 4: Validate circle fit
            if (!circle_fitter_.validateFit(circle_fit, cluster.size(), distance)) {
                RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 圆拟合验证失败", idx);
                continue;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 通过圆拟合验证", idx);
            
            // Step 5: Extract descriptor (fd_result, coverage already computed above)
            double timestamp = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            auto descriptor = descriptor_extractor_.extract(
                cluster, processed_cluster, circle_fit, fd_result, coverage, timestamp);
            
            // Step 8: Compute confidence
            double confidence = descriptor_extractor_.computeConfidence(descriptor);
            
            RCLCPP_DEBUG(this->get_logger(), "簇 %zu: 置信度=%.3f", idx, confidence);
            
            // Step 9: Build detected reflector
            if (confidence >= min_confidence_) {
                DetectedReflector reflector;
                reflector.center = circle_fit.center;
                reflector.diameter = 2 * circle_fit.radius;
                reflector.confidence = confidence;
                reflector.point_count = cluster.size();
                reflector.mean_intensity = descriptor.mean_intensity;
                
                reflectors.push_back(reflector);
                
                RCLCPP_INFO(this->get_logger(), "检测到反光柱 %zu: 中心(%.3f,%.3f), 直径=%.3fm, 置信度=%.3f",
                           reflectors.size(), reflector.center.x, reflector.center.y,
                           reflector.diameter, reflector.confidence);
            }
        }
        
        if (reflectors.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "未检测到有效反光柱");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "检测到 %zu 个反光柱", reflectors.size());
        
        // Phase 4: Update tracking
        std::vector<LegacyTrackedReflector> tracked_reflectors;
        if (enable_tracking_) {
            tracked_reflectors = tracker_.update(reflectors, scan_msg->header.stamp);
            
            if (!tracked_reflectors.empty()) {
                RCLCPP_INFO(this->get_logger(), "追踪到 %zu 个反光柱 (带ID)", tracked_reflectors.size());
            }
        }
        
        // Publish landmarks (use tracked if available, otherwise use detected)
        if (enable_tracking_ && !tracked_reflectors.empty()) {
            publishLandmarks(tracked_reflectors, scan_msg);
            publishMarkers(tracked_reflectors, scan_msg);
        } else {
            publishLandmarks(reflectors, scan_msg);
            publishMarkers(reflectors, scan_msg);
        }
        
        // Publish debug markers
        // publishDebugMarkers(filtered_points, clusters, reflectors, scan_msg);
        
    }
    
    std::vector<Point> convertScanToPoints(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        
        std::vector<Point> points;
        
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            // Skip invalid ranges
            if (scan_msg->ranges[i] < scan_msg->range_min ||
                scan_msg->ranges[i] > scan_msg->range_max ||
                !std::isfinite(scan_msg->ranges[i])) {
                continue;
            }
            
            // Convert polar to cartesian
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double range = scan_msg->ranges[i];
            
            Point point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            
            // Get intensity if available
            if (i < scan_msg->intensities.size()) {
                point.intensity = scan_msg->intensities[i];
            } else {
                point.intensity = 0.0; // No intensity data
            }
            
            points.push_back(point);
        }
        
        return points;
    }
    
    std::vector<Point> filterByIntensity(const std::vector<Point>& points, double threshold) {
        std::vector<Point> filtered;
        filtered.reserve(points.size());
        
        for (const auto& point : points) {
            if (point.intensity >= threshold) {
                filtered.push_back(point);
            }
        }
        
        return filtered;
    }
    
    Point computeCentroid(const std::vector<Point>& cluster) {
        Point centroid;
        if (cluster.empty()) {
            centroid.x = 0.0;
            centroid.y = 0.0;
            return centroid;
        }
        
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& point : cluster) {
            sum_x += point.x;
            sum_y += point.y;
        }
        
        centroid.x = sum_x / cluster.size();
        centroid.y = sum_y / cluster.size();
        return centroid;
    }
    
    void publishLandmarks(const std::vector<DetectedReflector>& reflectors,
                        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        cartographer_ros_msgs::msg::LandmarkList landmark_list;
        landmark_list.header = scan_msg->header;
        
        for (size_t i = 0; i < reflectors.size(); ++i) {
            cartographer_ros_msgs::msg::LandmarkEntry entry;
            
            // Create pose from center
            entry.tracking_from_landmark_transform.position.x = reflectors[i].center.x;
            entry.tracking_from_landmark_transform.position.y = reflectors[i].center.y;
            entry.tracking_from_landmark_transform.position.z = 0.0;
            entry.tracking_from_landmark_transform.orientation.w = 1.0;
            entry.tracking_from_landmark_transform.orientation.x = 0.0;
            entry.tracking_from_landmark_transform.orientation.y = 0.0;
            entry.tracking_from_landmark_transform.orientation.z = 0.0;
            
            // Set weights based on confidence
            entry.translation_weight = reflectors[i].confidence * 100.0;
            entry.rotation_weight = reflectors[i].confidence * 100.0;
            
            // Use confidence as ID for tracking
            entry.id = "R_" + std::to_string(i);
            
            landmark_list.landmarks.push_back(entry);
        }
        
        landmark_pub_->publish(landmark_list);
    }
    
    void publishLandmarks(const std::vector<LegacyTrackedReflector>& reflectors,
                        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        cartographer_ros_msgs::msg::LandmarkList landmark_list;
        landmark_list.header = scan_msg->header;
        
        for (const auto& reflector : reflectors) {
            cartographer_ros_msgs::msg::LandmarkEntry entry;
            
            // Create pose from center
            entry.tracking_from_landmark_transform.position.x = reflector.position.x;
            entry.tracking_from_landmark_transform.position.y = reflector.position.y;
            entry.tracking_from_landmark_transform.position.z = 0.0;
            entry.tracking_from_landmark_transform.orientation.w = 1.0;
            entry.tracking_from_landmark_transform.orientation.x = 0.0;
            entry.tracking_from_landmark_transform.orientation.y = 0.0;
            entry.tracking_from_landmark_transform.orientation.z = 0.0;
            
            // Set weights based on confidence
            entry.translation_weight = reflector.confidence * 100.0;
            entry.rotation_weight = reflector.confidence * 100.0;
            
            // Use tracking ID
            entry.id = "R_" + std::to_string(reflector.id);
            
            landmark_list.landmarks.push_back(entry);
        }
        
        landmark_pub_->publish(landmark_list);
    }
    
    void publishMarkers(const std::vector<DetectedReflector>& reflectors,
                     const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.resize(reflectors.size());
        
        for (size_t i = 0; i < reflectors.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = scan_msg->header;
            marker.ns = "reflective_posts";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position
            marker.pose.position.x = reflectors[i].center.x;
            marker.pose.position.y = reflectors[i].center.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            
            // Scale (diameter based)
            marker.scale.x = reflectors[i].diameter;
            marker.scale.y = reflectors[i].diameter;
            marker.scale.z = 0.5;
            
            // Color based on confidence
            double confidence = reflectors[i].confidence;
            marker.color.r = 0.0;
            marker.color.g = confidence;
            marker.color.b = 1.0 - confidence;
            marker.color.a = 0.8;
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            
            marker_array.markers[i] = marker;
        }
        
        marker_pub_->publish(marker_array);
    }
    
    void publishMarkers(const std::vector<LegacyTrackedReflector>& reflectors,
                     const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.resize(reflectors.size());
        
        for (size_t i = 0; i < reflectors.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = scan_msg->header;
            marker.ns = "tracked_reflectors";
            marker.id = reflectors[i].id;  // Use tracking ID
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position
            marker.pose.position.x = reflectors[i].position.x;
            marker.pose.position.y = reflectors[i].position.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            
            // Scale (fixed 7cm diameter)
            marker.scale.x = 0.07;
            marker.scale.y = 0.07;
            marker.scale.z = 0.5;
            
            // Color based on confidence (tracked reflectors are green)
            double confidence = reflectors[i].confidence;
            marker.color.r = 0.0;
            marker.color.g = 0.5 + 0.5 * confidence;
            marker.color.b = 0.0;
            marker.color.a = 0.9;
            
            // Add text label with ID
            marker.text = "ID:" + std::to_string(reflectors[i].id);
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            marker_array.markers[i] = marker;
        }
        
        marker_pub_->publish(marker_array);
    }
    
    void publishDebugMarkers(const std::vector<Point>& filtered_points,
                          const std::vector<std::vector<Point>>& clusters,
                          const std::vector<DetectedReflector>& /*reflectors*/,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Add markers for filtered points (yellow)
        for (size_t i = 0; i < filtered_points.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = scan_msg->header;
            marker.ns = "filtered_points";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = filtered_points[i].x;
            marker.pose.position.y = filtered_points[i].y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            marker_array.markers.push_back(marker);
        }
        
        // Add markers for clusters (different colors per cluster)
        size_t marker_id = filtered_points.size();
        for (size_t c = 0; c < clusters.size(); ++c) {
            float hue = static_cast<float>(c) / std::max(1.0f, static_cast<float>(clusters.size()));
            
            for (size_t i = 0; i < clusters[c].size(); ++i) {
                visualization_msgs::msg::Marker marker;
                marker.header = scan_msg->header;
                marker.ns = "cluster_" + std::to_string(c);
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                marker.pose.position.x = clusters[c][i].x;
                marker.pose.position.y = clusters[c][i].y;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.w = 1.0;
                
                marker.scale.x = 0.03;
                marker.scale.y = 0.03;
                marker.scale.z = 0.03;
                
                // HSV to RGB conversion
                float r, g, b;
                hsvToRgb(hue, 1.0f, 1.0f, r, g, b);
                marker.color.r = r;
                marker.color.g = g;
                marker.color.b = b;
                marker.color.a = 0.7;
                
                marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                
                marker_array.markers.push_back(marker);
            }
        }
        
        debug_pub_->publish(marker_array);
    }
    
    void hsvToRgb(float h, float s, float v, float& r, float& g, float& b) {
        if (s == 0.0f) {
            r = g = b = v;
            return;
        }
        
        h *= 6.0f;
        int i = static_cast<int>(std::floor(h));
        float f = h - i;
        float p = v * (1.0f - s);
        float q = v * (1.0f - s * f);
        float t = v * (1.0f - s * (1.0f - f));
        
        switch (i) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            case 5: r = v; g = p; b = q; break;
            default: r = g = b = v; break;
        }
    }
    
    void publishCompensatedCloud(const std::vector<Point>& points,
                                 const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = scan_msg->header;
        cloud_msg.height = 1;
        cloud_msg.width = points.size();
        
        // Define point fields: x, y, z, intensity
        cloud_msg.fields.resize(4);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;
        
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;
        
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;
        
        cloud_msg.fields[3].name = "intensity";
        cloud_msg.fields[3].offset = 12;
        cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[3].count = 1;
        
        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 16;  // 4 fields * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        
        // Allocate data buffer
        cloud_msg.data.resize(cloud_msg.row_step);
        
        // Fill point cloud data
        for (size_t i = 0; i < points.size(); ++i) {
            uint8_t* ptr = &cloud_msg.data[i * cloud_msg.point_step];
            
            // x coordinate
            float x = static_cast<float>(points[i].x);
            std::memcpy(ptr + 0, &x, sizeof(float));
            
            // y coordinate
            float y = static_cast<float>(points[i].y);
            std::memcpy(ptr + 4, &y, sizeof(float));
            
            // z coordinate (always 0)
            float z = 0.0f;
            std::memcpy(ptr + 8, &z, sizeof(float));
            
            // intensity
            float intensity = static_cast<float>(points[i].intensity);
            std::memcpy(ptr + 12, &intensity, sizeof(float));
        }
        
        compensated_cloud_pub_->publish(cloud_msg);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    // Publishers
    rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr landmark_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr compensated_cloud_pub_;
    
    // Processing modules (New Pipeline)
    FixedDBSCAN fixed_dbscan_;                      // Phase 1: Fixed DBSCAN
    FractalDimensionCalculator fd_calculator_;      // Phase 2: Fractal dimension filtering
    PCAShapeClassifier pca_classifier_;             // Phase 2: PCA shape classification
    ImprovedInterpolationCompensator improved_interpolator_;  // Phase 2: Improved interpolation
    CircleFitter circle_fitter_;                    // Phase 3: Circle fitting
    PracticalDescriptorExtractor descriptor_extractor_;  // Phase 3: Descriptor extraction
    GeometricValidator geometric_validator_;       // Legacy validation (optional)
    ReflectorTracker tracker_;                     // Phase 4: Tracking
    
    // Parameters
    std::string classification_method_;
    double raw_intensity_threshold_;
    bool enable_interpolation_;
    bool enable_tracking_;
    double min_confidence_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ReflectorNoiseHandlingNode>();
    
    RCLCPP_INFO(node->get_logger(), "启动反光柱噪声处理节点 (固定阈值+追踪版本)");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}