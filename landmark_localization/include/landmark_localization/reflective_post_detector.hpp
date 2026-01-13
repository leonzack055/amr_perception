#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <functional>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <numeric>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include "common/reflector_common.hpp"
namespace landmark_localization
{

constexpr int MAX_TRACKS = 50;
constexpr int MAX_DETECTIONS = 30;
constexpr int TRACK_HISTORY = 20;
constexpr float MATCH_DISTANCE = 0.2f;
constexpr int CONFIRM_FRAMES = 2;
//constexpr int MAX_AGE = 5;

// 轨迹状态
enum TrackState {
    TENTATIVE = 0,
    CONFIRMED = 1,  
    LOST = 2
};

// 跟踪点结构
struct TrackPoint {
    float x, y;
    float timestamp;
    float t_w;
    float r_w;
    float confidence;
    float diameter;
    int id;
    
    TrackPoint(float x_ = 0, float y_ = 0, float ts_ = 0, float t_ = 0, float r_ = 0,float conf_ = 1.0f,float d_ = 0, int id_ = -1)
        : x(x_), y(y_), timestamp(ts_), t_w(t_), r_w(r_), confidence(conf_), diameter(d_), id(id_) {}
};

// 轨迹类
class X5Track {
public:
    int id;
    TrackState state;
    int age;
    int total_visible_count;
    int consecutive_invisible_count;
    
    std::array<TrackPoint, TRACK_HISTORY> history;
    int history_count;
    
    float velocity_x, velocity_y;
    float motion_consistency;

    int max_age;
    
    // 默认构造函数
    X5Track(int max_age_val = 5) 
        : id(-1), state(TENTATIVE), age(0), 
          total_visible_count(0), consecutive_invisible_count(0),
          history_count(0), velocity_x(0), velocity_y(0), motion_consistency(1.0f), max_age(max_age_val){}
    
    // 参数化构造函数
    X5Track(int track_id, const TrackPoint& first_point,int max_age_val = 5) 
        : id(track_id), state(TENTATIVE), age(1), 
          total_visible_count(1), consecutive_invisible_count(0),
          history_count(1), velocity_x(0), velocity_y(0), motion_consistency(1.0f), max_age(max_age_val) {
        history[0] = first_point;
    }
    
    void predictPosition(float& pred_x, float& pred_y, float dt = 0.033f) const {
        if (history_count == 0) {
            pred_x = 0; pred_y = 0;
            return;
        }
        
        const auto& latest = history[history_count-1];
        
        if (history_count == 1 || consecutive_invisible_count > 0) {
            pred_x = latest.x;
            pred_y = latest.y;
            return;
        }
        
        if (history_count >= 2) {
            const auto& prev = history[history_count-2];
            float dx = latest.x - prev.x;
            float dy = latest.y - prev.y;
            float prev_dt = latest.timestamp - prev.timestamp;
            
            if (prev_dt > 1e-6f) {
                float vx = dx / prev_dt;
                float vy = dy / prev_dt;
                
                pred_x = latest.x + vx * dt;
                pred_y = latest.y + vy * dt;
                
                if (history_count >= 3) {
                    const auto& prev2 = history[history_count-3];
                    float dx_old = prev.x - prev2.x;
                    float dy_old = prev.y - prev2.y;
                    float dt_old = prev.timestamp - prev2.timestamp;
                    
                    if (dt_old > 1e-6f) {
                        float vx_old = dx_old / dt_old;
                        float vy_old = dy_old / dt_old;
                        
                        float ax = (vx - vx_old) / (prev_dt + dt_old) * 0.5f;
                        float ay = (vy - vy_old) / (prev_dt + dt_old) * 0.5f;
                        
                        pred_x += 0.5f * ax * dt * dt;
                        pred_y += 0.5f * ay * dt * dt;
                    }
                }
                return;
            }
        }
        
        pred_x = latest.x;
        pred_y = latest.y;
    }
    
    void update(const TrackPoint& new_point) {
        if (history_count > 0) {
            updateMotionConsistency(new_point);
        }
        
        if (history_count < TRACK_HISTORY) {
            history[history_count++] = new_point;
        } else {
            for (int i = 0; i < TRACK_HISTORY-1; ++i) {
                history[i] = history[i+1];
            }
            history[TRACK_HISTORY-1] = new_point;
        }
        
        if (history_count >= 2) {
            const auto& latest = history[history_count-1];
            const auto& prev = history[history_count-2];
            float dt = latest.timestamp - prev.timestamp;
            if (dt > 1e-6f) {
                velocity_x = (latest.x - prev.x) / dt;
                velocity_y = (latest.y - prev.y) / dt;
            }
        }
        
        age++;
        total_visible_count++;
        consecutive_invisible_count = 0;
        
        if (state == TENTATIVE && total_visible_count >= CONFIRM_FRAMES) {
            state = CONFIRMED;
        }
    }
    
    void markMissed() {
        consecutive_invisible_count++;
        if (consecutive_invisible_count > max_age) {
            state = LOST;
        }
    }
    
    float getAdaptiveThreshold() const {
        float base_threshold = MATCH_DISTANCE;
        
        float speed = std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y);
        if (speed > 1.0f) {
            base_threshold *= 1.3f;
        }
        
        if (state == TENTATIVE) {
            base_threshold *= 1.5f;
        }
        
        if (motion_consistency < 0.7f) {
            base_threshold *= 1.2f;
        }
        
        return base_threshold;
    }
    
    bool isConfirmed() const { return state == CONFIRMED; }
    bool isLost() const { return state == LOST; }
    bool shouldRemove() const {
        return isLost() || (state == TENTATIVE && consecutive_invisible_count > 5);
    }
    
    TrackPoint getLatestPoint() const {
        if (history_count > 0) {
            return history[history_count-1];
        }
        return TrackPoint();
    }

private:
    void updateMotionConsistency(const TrackPoint& new_point) {
        if (history_count < 2) return;
        
        const auto& latest = history[history_count-1];
        const auto& prev = history[history_count-2];
        
        float hist_dx = latest.x - prev.x;
        float hist_dy = latest.y - prev.y;
        float curr_dx = new_point.x - latest.x;
        float curr_dy = new_point.y - latest.y;
        
        float dot_product = hist_dx * curr_dx + hist_dy * curr_dy;
        float hist_len = std::sqrt(hist_dx*hist_dx + hist_dy*hist_dy);
        float curr_len = std::sqrt(curr_dx*curr_dx + curr_dy*curr_dy);
        
        if (hist_len < 1e-6f || curr_len < 1e-6f) {
            motion_consistency = 1.0f;
        } else {
            float cosine_sim = dot_product / (hist_len * curr_len);
            motion_consistency = (cosine_sim + 1.0f) / 2.0f;
        }
    }
};

// 主跟踪器
class X5MOTTracker {
private:
    std::array<std::unique_ptr<X5Track>, MAX_TRACKS> tracks_;
    int active_track_count_;
    int next_id_;
    float last_timestamp_;
    int max_age_;
    
public:
    X5MOTTracker(int max_age_val_ = 5) : active_track_count_(0), next_id_(0), last_timestamp_(0),max_age_(max_age_val_) {
        // 显式初始化所有指针为nullptr
        for (auto& track : tracks_) {
            track.reset();
        }
    }
    
    std::vector<TrackPoint> processFrame(const std::vector<TrackPoint>& detections, float timestamp) {
        if (detections.empty() && active_track_count_ == 0) {
            return std::vector<TrackPoint>();
        }
        
        float dt = (last_timestamp_ > 0) ? (timestamp - last_timestamp_) : 0.033f;
        last_timestamp_ = timestamp;
        
        auto matches = robustDataAssociation(detections, dt);
        updateMatchedTracks(matches, detections);
        createNewTracks(matches, detections);
        manageTracks();
        
        return prepareOutput();
    }
    
    void reset() {
        for (auto& track : tracks_) {
            track.reset();
        }
        active_track_count_ = 0;
        next_id_ = 0;
        last_timestamp_ = 0;
    }
    
    int getActiveTrackCount() const { return active_track_count_; }

private:
    struct MatchResult {
        int matches[MAX_DETECTIONS][2];
        int match_count;
        bool unmatched_dets[MAX_DETECTIONS];
        bool unmatched_tracks[MAX_TRACKS];
        
        MatchResult() : match_count(0) {
            std::fill_n(unmatched_dets, MAX_DETECTIONS, true);
            std::fill_n(unmatched_tracks, MAX_TRACKS, true);
        }
    };
    
    MatchResult robustDataAssociation(const std::vector<TrackPoint>& detections, float dt) {
        MatchResult result;
        
        if (active_track_count_ == 0 || detections.empty()) {
            for (int i = 0; i < static_cast<int>(detections.size()) && i < MAX_DETECTIONS; ++i) {
                result.unmatched_dets[i] = true;
            }
            return result;
        }
        
        std::vector<std::tuple<float, int, int>> candidates;
        
        for (int d_idx = 0; d_idx < static_cast<int>(detections.size()) && d_idx < MAX_DETECTIONS; ++d_idx) {
            for (int t_idx = 0; t_idx < MAX_TRACKS; ++t_idx) {
                if (tracks_[t_idx] == nullptr || !tracks_[t_idx]->isConfirmed()) continue;
                if (!result.unmatched_tracks[t_idx]) continue;
                
                float cost = calculateMatchCost(detections[d_idx], t_idx, dt);
                float threshold = tracks_[t_idx]->getAdaptiveThreshold();
                
                if (cost < threshold) {
                    candidates.emplace_back(cost, d_idx, t_idx);
                }
            }
        }
        
        std::sort(candidates.begin(), candidates.end(), 
                 [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });
        
        for (const auto& candidate : candidates) {
            float cost = std::get<0>(candidate);
            int d_idx = std::get<1>(candidate);
            int t_idx = std::get<2>(candidate);
            
            if (result.unmatched_dets[d_idx] && result.unmatched_tracks[t_idx]) {
                result.matches[result.match_count][0] = d_idx;
                result.matches[result.match_count][1] = t_idx;
                result.match_count++;
                
                result.unmatched_dets[d_idx] = false;
                result.unmatched_tracks[t_idx] = false;
            }
        }
        
        return result;
    }
    
    float calculateMatchCost(const TrackPoint& detection, int track_idx, float dt) {
        if (tracks_[track_idx] == nullptr) return 9999.0f;
        
        const auto& track = *tracks_[track_idx];
        
        float pred_x, pred_y;
        track.predictPosition(pred_x, pred_y, dt);
        float dx = pred_x - detection.x;
        float dy = pred_y - detection.y;
        float distance_cost = std::sqrt(dx*dx + dy*dy);
        
        float motion_cost = 0.0f;
        if (track.history_count >= 2) {
            const auto& latest = track.getLatestPoint();
            float curr_dx = detection.x - latest.x;
            float curr_dy = detection.y - latest.y;
            
            float speed = std::sqrt(track.velocity_x*track.velocity_x + track.velocity_y*track.velocity_y);
            if (speed > 0.1f) {
                float dot = track.velocity_x * curr_dx + track.velocity_y * curr_dy;
                float vel_len = speed;
                float curr_len = std::sqrt(curr_dx*curr_dx + curr_dy*curr_dy);
                
                if (curr_len > 1e-6f) {
                    float cosine = dot / (vel_len * curr_len);
                    motion_cost = (1.0f - cosine) * 0.5f;
                }
            }
        }
        
        return distance_cost + motion_cost * 0.8f;
    }
    
    void updateMatchedTracks(const MatchResult& matches, 
                           const std::vector<TrackPoint>& detections) {
        for (int i = 0; i < matches.match_count; ++i) {
            int det_idx = matches.matches[i][0];
            int track_idx = matches.matches[i][1];
            
            if (tracks_[track_idx] == nullptr) continue;
            
            TrackPoint updated_point = detections[det_idx];
            updated_point.id = tracks_[track_idx]->id;
            tracks_[track_idx]->update(updated_point);
        }
        
        for (int t_idx = 0; t_idx < MAX_TRACKS; ++t_idx) {
            if (tracks_[t_idx] != nullptr && matches.unmatched_tracks[t_idx]) {
                tracks_[t_idx]->markMissed();
            }
        }
    }
    
    void createNewTracks(const MatchResult& matches, 
                        const std::vector<TrackPoint>& detections) {
        for (int d_idx = 0; d_idx < static_cast<int>(detections.size()) && d_idx < MAX_DETECTIONS; ++d_idx) {
            if (matches.unmatched_dets[d_idx] && active_track_count_ < MAX_TRACKS) {
                for (int i = 0; i < MAX_TRACKS; ++i) {
                    if (tracks_[i] == nullptr) {
                        tracks_[i] = std::make_unique<X5Track>(next_id_++, detections[d_idx],max_age_);
                        active_track_count_++;
                        break;
                    }
                }
            }
        }
    }
    
    void manageTracks() {
        for (int i = 0; i < MAX_TRACKS; ++i) {
            if (tracks_[i] != nullptr && tracks_[i]->shouldRemove()) {
                tracks_[i].reset();
                active_track_count_--;
            }
        }
    }
    
    std::vector<TrackPoint> prepareOutput() {
        std::vector<TrackPoint> output;
        
        for (int i = 0; i < MAX_TRACKS; ++i) {
            if (tracks_[i] != nullptr && tracks_[i]->consecutive_invisible_count == 0) {
                output.push_back(tracks_[i]->getLatestPoint());
            }
        }
        
        return output;
    }
};

// 轨迹平滑器
class TrajectorySmoother {
private:
    struct SmoothState {
        float x, y;
        float velocity_x, velocity_y;
        int update_count;
        
        SmoothState() : x(0), y(0), velocity_x(0), velocity_y(0), update_count(0) {}
    };
    std::array<SmoothState, MAX_TRACKS> states_;
    
public:
    TrajectorySmoother() {
        // 数组会自动初始化
    }
    
    void smoothTrajectory(TrackPoint& point) {
        int id = point.id;
        if (id < 0 || id >= MAX_TRACKS) return;
        
        auto& state = states_[id];
        
        if (state.update_count == 0) {
            state.x = point.x;
            state.y = point.y;
            state.update_count = 1;
        } else {
            float speed = std::sqrt(state.velocity_x*state.velocity_x + 
                                  state.velocity_y*state.velocity_y);
            float alpha = (speed > 1.0f) ? 0.8f : 0.6f;
            
            state.velocity_x = alpha * state.velocity_x + (1-alpha) * (point.x - state.x);
            state.velocity_y = alpha * state.velocity_y + (1-alpha) * (point.y - state.y);
            
            state.x = alpha * state.x + (1-alpha) * point.x;
            state.y = alpha * state.y + (1-alpha) * point.y;
            state.update_count++;
            
            point.x = state.x;
            point.y = state.y;
        }
    }
    
    void resetTrack(int id) {
        if (id >= 0 && id < MAX_TRACKS) {
            states_[id] = SmoothState();
        }
    }
    
    void resetAll() {
        for (auto& state : states_) {
            state = SmoothState();
        }
    }
};
class FrameRateController {
private:
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::microseconds target_frame_duration_;
    
public:
    FrameRateController(int target_fps = 30) {
        setTargetFPS(target_fps);
        last_frame_time_ = std::chrono::steady_clock::now();
    }
    
    void setTargetFPS(int fps) {
        target_frame_duration_ = std::chrono::microseconds(1000000 / fps);
    }
    
    void sleepForFrameRate() {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = current_time - last_frame_time_;
        
        if (elapsed < target_frame_duration_) {
            auto sleep_time = target_frame_duration_ - elapsed;
            std::this_thread::sleep_for(sleep_time);
        }
        
        last_frame_time_ = std::chrono::steady_clock::now();
    }
    
    // 获取实际帧率（用于监控）
    float getCurrentFPS() {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            current_time - last_frame_time_);
        return 1000000.0f / elapsed.count();
    }
};

// 并查集用于优化DBSCAN
class UnionFind {
private:
    std::vector<int> parent;
    std::vector<int> rank;
public:
    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }
    
    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }
    
    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }
};
// 点云数据结构
struct Point {
    double x, y;    // 坐标
    double intensity; // 强度值
    Point() : x(0), y(0), intensity(0){}
    Point(double x, double y) : x(x), y(y), intensity(intensity){}
    
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }
    
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
    
    Point operator/(double scalar) const {
        return Point(x / scalar, y / scalar);
    }
    
    Point& operator+=(const Point& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    double norm() const{
        return std::sqrt(x*x + y*y);
    }
    double squaredNorm() const {
        return x*x + y*y;
    }
};
struct GaussianParams {
    double A;     // 振幅（圆心处强度）
    double sigma; // 标准差（衰减速度）
};

class ReflectorCenterDetector {
private:
    double expected_radius_;
    
public:
    ReflectorCenterDetector(double radius = 0.03) 
        : expected_radius_(radius) {}
    
    bool detectReflectorCenter(const std::vector<Point>& points, Point& center, Point& radar_origin, double& confidence) {
        if (points.empty()) {
            confidence = 0.0;
            return false;
        }
        
        // 1. 快速点数检查
        if (points.size() < 8) {
            confidence = 0.1 * points.size() / 8.0;
            return false;
        }
        
        // 2. 点数稳定性处理
        std::vector<Point> stable_points = points;
        /*if (points.size() > 20) {
            // 均匀采样到20个点，保持计算稳定性
            stable_points.clear();
            double step = static_cast<double>(points.size()) / 20.0;
            for (int i = 0; i < 20; ++i) {
                int index = static_cast<int>(i * step);
                if (index < points.size()) {
                    stable_points.push_back(points[index]);
                }
            }
        }*/
        
        // 3. 强度归一化 - 减少绝对强度影响
        normalizeIntensities(stable_points);
        
        // 初始估计
        Point initial_center = computeIntensityWeightedCentroid(stable_points);
        
        // 快速径向强度分析
        center = refineCenterWithRadialGaussianFitting(stable_points, initial_center, confidence);
        
        // 计算实际圆心
        center = calculateActualCenter(stable_points, center, expected_radius_, radar_origin);
        
        return confidence > 0.3;
    }
    
private:
    // 强度归一化
    void normalizeIntensities(std::vector<Point>& points) {
        if (points.empty()) return;
        
        double max_intensity = points[0].intensity;
        for (const auto& p : points) {
            if (p.intensity > max_intensity) {
                max_intensity = p.intensity;
            }
        }
        
        if (max_intensity > 1e-6) {
            for (auto& p : points) {
                p.intensity /= max_intensity;
            }
        }
    }
    
    // 优化后的强度加权重心计算
    Point computeIntensityWeightedCentroid(const std::vector<Point>& points) {
        // 使用循环展开和累加优化
        const size_t size = points.size();
        const size_t block_size = 4;
        const size_t blocks = size / block_size;
        
        double sum_x = 0, sum_y = 0, sum_weight = 0;
        
        // 主循环 - 循环展开
        for (size_t i = 0; i < blocks; ++i) {
            const size_t base = i * block_size;
            const Point& p0 = points[base];
            const Point& p1 = points[base + 1];
            const Point& p2 = points[base + 2];
            const Point& p3 = points[base + 3];
            
            sum_x += p0.x * p0.intensity + p1.x * p1.intensity + 
                    p2.x * p2.intensity + p3.x * p3.intensity;
            sum_y += p0.y * p0.intensity + p1.y * p1.intensity + 
                    p2.y * p2.intensity + p3.y * p3.intensity;
            sum_weight += p0.intensity + p1.intensity + p2.intensity + p3.intensity;
        }
        
        // 处理剩余点
        for (size_t i = blocks * block_size; i < size; ++i) {
            const Point& p = points[i];
            sum_x += p.x * p.intensity;
            sum_y += p.y * p.intensity;
            sum_weight += p.intensity;
        }
        
        Point centroid;
        if (sum_weight > 1e-6) {
            centroid.x = sum_x / sum_weight;
            centroid.y = sum_y / sum_weight;
        } else {
            // 快速平均值计算
            double sum_x_simple = 0, sum_y_simple = 0;
            for (const auto& p : points) {
                sum_x_simple += p.x;
                sum_y_simple += p.y;
            }
            centroid.x = sum_x_simple / size;
            centroid.y = sum_y_simple / size;
        }
        centroid.intensity = 0;
        double radar_distance = sqrt(centroid.x * centroid.x + centroid.y * centroid.y);
        if (radar_distance < 1.0) { // 近距离时
            // 所有点的Y平均值
            double sum_y_simple = 0;
            for (const auto& p : points) {
                sum_y_simple += p.y;
            }
            centroid.y = sum_y_simple / points.size();
        }
        
        return centroid;
    }

    // 优化后的径向高斯拟合
    Point refineCenterWithRadialGaussianFitting(const std::vector<Point>& points, 
                                              const Point& initial_center,
                                              double& confidence) {
        // 预分配内存，避免动态分配
        const int max_points = 20; // 限制处理点数，提高稳定性
        double distances[max_points];
        double intensities[max_points];
        int valid_count = 0;
        
        const double max_distance = expected_radius_ * 1.2;
        const double max_distance_sq = max_distance * max_distance;
        
        // 快速收集有效点
        for (const auto& p : points) {
            if (valid_count >= max_points) break;
            
            double dx = p.x - initial_center.x;
            double dy = p.y - initial_center.y;
            double distance_sq = dx * dx + dy * dy;
            
            if (distance_sq < max_distance_sq) {
                distances[valid_count] = sqrt(distance_sq);
                intensities[valid_count] = p.intensity;
                valid_count++;
            }
        }
        
        if (valid_count < 5) {
            confidence = 0.2;
            return initial_center;
        }
        
        // 稳健的高斯拟合
        GaussianParams params;
        if (robustFastGaussianFit(distances, intensities, valid_count, params)) {
            // 计算拟合置信度
            confidence = computeStableGaussianConfidence(distances, intensities, valid_count, params);
            
            // 快速圆心精化
            return fastRecomputeCenter(points, initial_center, params);
        }
        
        confidence = 0.15;
        return initial_center;
    }
    
    // 稳健的快速高斯拟合
    bool robustFastGaussianFit(const double* distances, const double* intensities, 
                              int count, GaussianParams& params) {
        if (count < 3) return false;
        
        // 使用加权拟合，提高稳定性
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        double sum_w = 0;
        int valid_count = 0;
        
        for (int i = 0; i < count; i++) {
            if (intensities[i] > 1e-6) {
                double r_sq = distances[i] * distances[i];
                double log_intensity = log(intensities[i]);
                
                // 距离加权：靠近中心的点权重更高
                double weight = exp(-r_sq / (expected_radius_ * expected_radius_));
                
                sum_x += weight * r_sq;
                sum_y += weight * log_intensity;
                sum_xy += weight * r_sq * log_intensity;
                sum_x2 += weight * r_sq * r_sq;
                sum_w += weight;
                valid_count++;
            }
        }
        
        if (valid_count < 3) return false;
        
        double denominator = sum_w * sum_x2 - sum_x * sum_x;
        if (fabs(denominator) < 1e-10) return false;
        
        double b = (sum_w * sum_xy - sum_x * sum_y) / denominator;
        double a = (sum_y - b * sum_x) / sum_w;
        
        if (b >= 0) return false;
        
        params.A = exp(a);
        params.sigma = sqrt(-1.0 / (2 * b));
        
        // 参数合理性检查
        return params.sigma > 0.005 && params.sigma < expected_radius_ * 2 && params.A >0.8;
    }
    
    // 稳定的高斯拟合置信度计算
    double computeStableGaussianConfidence(const double* distances, const double* intensities,
                                         int count, const GaussianParams& params) {
        if (count < 3) return 0.0;
        
        double total_error = 0.0;
        double max_intensity = 0.0;
        int valid_points = 0;
        
        for (int i = 0; i < count; i++) {
            if (intensities[i] > 1e-6) {
                double predicted = params.A * exp(-distances[i] * distances[i] / 
                                                (2 * params.sigma * params.sigma));
                double error = fabs(intensities[i] - predicted) / (intensities[i] + 0.1); // 避免除零
                total_error += error;
                
                if (intensities[i] > max_intensity) {
                    max_intensity = intensities[i];
                }
                valid_points++;
            }
        }
        
        if (valid_points == 0) return 0.0;
        
        double mean_error = total_error / valid_points;
        
        // 综合置信度：基于拟合误差、强度分布和参数合理性
        double error_confidence = std::max(0.0, 1.0 - mean_error);
        double intensity_confidence = std::min(1.0, max_intensity);
        double distribution_confidence = evaluateDistributionQuality(distances, count);
        
        return 0.5 * error_confidence + 0.3 * intensity_confidence + 0.2 * distribution_confidence;
    }
    
    // 评估点分布质量
    double evaluateDistributionQuality(const double* distances, int count) {
        if (count < 3) return 0.0;
        
        double mean_distance = 0.0;
        for (int i = 0; i < count; i++) {
            mean_distance += distances[i];
        }
        mean_distance /= count;
        
        // 理想分布应该在期望半径附近
        double distribution_score = 1.0 - fabs(mean_distance - expected_radius_) / expected_radius_;
        return std::max(0.0, distribution_score);
    }
    
    // 快速圆心精化（减少搜索范围但提高稳定性）
    Point fastRecomputeCenter(const std::vector<Point>& points,
                            const Point& current_center,
                            const GaussianParams& params) {
        double best_x = current_center.x;
        double best_y = current_center.y;
        double best_likelihood = -1e10;
        
        // 两阶段搜索：先粗后精
        const double coarse_range = 0.008;
        const int coarse_steps = 4;
        
        // 粗搜索
        for (int i = -coarse_steps; i <= coarse_steps; i += 2) {
            for (int j = -coarse_steps; j <= coarse_steps; j += 2) {
                double test_x = current_center.x + i * coarse_range / coarse_steps;
                double test_y = current_center.y + j * coarse_range / coarse_steps;
                
                double likelihood = fastGaussianLikelihood(points, test_x, test_y, params);
                
                if (likelihood > best_likelihood) {
                    best_likelihood = likelihood;
                    best_x = test_x;
                    best_y = test_y;
                }
            }
        }
        
        // 精搜索
        const double fine_range = 0.005;
        const int fine_steps = 3;
        Point refined_center;
        refined_center.x = best_x;
        refined_center.y = best_y;
        
        for (int i = -fine_steps; i <= fine_steps; ++i) {
            for (int j = -fine_steps; j <= fine_steps; ++j) {
                double test_x = refined_center.x + i * fine_range / fine_steps;
                double test_y = refined_center.y + j * fine_range / fine_steps;
                
                double likelihood = fastGaussianLikelihood(points, test_x, test_y, params);
                
                if (likelihood > best_likelihood) {
                    best_likelihood = likelihood;
                    refined_center.x = test_x;
                    refined_center.y = test_y;
                }
            }
        }
        
        refined_center.intensity = params.A;
        return refined_center;
    }

    // 快速高斯似然计算（优化版本）
    double fastGaussianLikelihood(const std::vector<Point>& points,
                                double center_x, double center_y,
                                const GaussianParams& params) {
        double total_likelihood = 0;
        int count = 0;
        const double sigma_sq_2 = 2 * params.sigma * params.sigma;
        
        // 使用距离平方避免开方
        for (const auto& p : points) {
            double dx = p.x - center_x;
            double dy = p.y - center_y;
            double distance_sq = dx * dx + dy * dy;
            
            if (distance_sq < expected_radius_ * expected_radius_ * 9) {
                double predicted = params.A * exp(-distance_sq / sigma_sq_2);
                total_likelihood += -fabs(p.intensity - predicted); // 使用绝对值加速
                count++;
            }
        }
        
        return count > 0 ? total_likelihood / count : -1e10;
    }
    
    // 保持原有方法不变
    Point calculateActualCenter(const std::vector<Point>& points, const Point& reflection_center, 
                              double expected_radius, const Point& radar_origin) {
        Point actual_center = reflection_center;
        
        double dx = reflection_center.x - radar_origin.x;
        double dy = reflection_center.y - radar_origin.y;
        double distance_to_radar_sq = dx * dx + dy * dy;
        
        if (distance_to_radar_sq < 1e-10) {
            return actual_center;
        }
        
        double distance_to_radar = sqrt(distance_to_radar_sq);
        double dir_x = dx / distance_to_radar;
        double dir_y = dy / distance_to_radar;
        
        actual_center.x = reflection_center.x + dir_x * expected_radius;
        actual_center.y = reflection_center.y + dir_y * expected_radius;
        actual_center.intensity = 0;
        
        return actual_center;
    }
};
// 纯 C++ 数据结构，替代 ROS 类型
struct LaserScan
{
  std::string header = "";
  std::vector<float> ranges;
  std::vector<float> intensities;
  float angle_min = 0.0f;
  float angle_max = 0.0f;
  float angle_increment = 0.0f;

  float time_increment = 0.0f;
  float scan_time = 0.0f; // 这个scan_time表示什么？

  float range_min = 0.0f;
  float range_max = 0.0f;
};

class ReflectivePostDetector
{
public:
  ReflectivePostDetector(int intensity_threshold = 1000,int max_age_launch_ = 5);
  ~ReflectivePostDetector();

  // Detect reflective posts from laser scan (非 ROS 环境可直接调用)
  std::vector<ReflectivePost> detect(const LaserScan & scan);
  std::vector<Detection> detect_circles(const LaserScan & scan);
  std::vector<TrackPoint> trackWithMOT(const std::vector<TrackPoint>& detections_);


  // 参数声明
  int intensity_threshold_use = 1000;
  double cluster_eps = 0.5;
  int min_cluster_points = 8;
  double diameter_min = 0.04;
  double diameter_max = 0.08;
  double residual_avg_threshold = 0.01;
  double residual_std_threshold = 0.005;
  double residual_max_threshold = 0.02;
  int stat_mean_k = 5;
  double stat_std_threshold = 1.0;
  int max_history_age = 3;
  double match_distance_threshold = 0.1;
  double arc_threshold = 0.1;
  double max_arc_feature = 30.0;
  int arc_min_points = 4;
  // double direction_tolerance = 0.785;
  double residual_real = 0.032;
  double sensitivity = 2.0;
  double maxError = 1.0;
  double maxangleError = 0.1;
  double paramError = 0.001;
  double distance_decay = 3.0;
  double landmark_rotation_weight = 1e2;
  double landmark_translation_weight = 1e5;
  int max_age_param_ = 5;

private:
  FrameRateController frame_controller{30};
  // 定义基本数据结构
  struct WeightedCircle {
      double center_x, center_y;
      double weight_translation;  // 平移权重
      double weight_rotation;     // 旋转权重
      double fit_error;           // 拟合误差
      int point_count;            // 使用点数
  };
  struct WeightParams {
      double ideal_point_count = 5.0;    // 理想点数
      double error_threshold = 0.001;    // 误差阈值
      double distance_decay = 10.0;      // 距离衰减系数
      double min_angle_diversity = 0.5;  // 最小角度多样性阈值
  };

  float distanceMax(std::vector<Point> &vec_) {
    double maxDist = 0;
    int n = vec_.size();

    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        double dist = (vec_[i].x - vec_[j].x) * (vec_[i].x - vec_[j].x) +
                      (vec_[i].y - vec_[j].y) * (vec_[i].y - vec_[j].y);
        if (dist > maxDist) {
          maxDist = dist;
        }
      }
    }

    std::cout << "&&&&&&&&&&&&&&&&" << maxDist << "&&&&&&&&&&&&&&&&"
              << std::endl;
    return maxDist;
  }

  /**
  * 计算角度多样性指标 (0-1之间)
  */
  double calculateAngleDiversity(const std::vector<Point>& points, 
                                  const Point& center) {
      if (points.size() < 3) return 0.5; // 点数太少，给中等权重
      
      std::vector<double> angles;
      for (const auto& p : points) {
          double dx = p.x - center.x;
          double dy = p.y - center.y;
          double angle = std::atan2(dy, dx);
          angles.push_back(angle);
      }
      
      std::sort(angles.begin(), angles.end());
      
      // 计算最大角度间隔
      double max_gap = 0;
      for (size_t i = 0; i < angles.size(); ++i) {
          double gap = angles[(i + 1) % angles.size()] - angles[i];
          if (gap < 0) gap += 2 * M_PI;
          max_gap = std::max(max_gap, gap);
      }
      
      // 最大间隔越小，分布越均匀，权重越高
      return 1.0 - (max_gap / (2 * M_PI));
  }

  /**
  * 为计算出的圆心赋权重
  */
  WeightedCircle assignWeightsToCircle(
      const std::vector<Point>& points,      // 使用的点云
      const Point& calculated_center,        // 计算出的圆心
      double fit_error,                      // 拟合误差
      const Point& vehicle_position,         // 车辆当前位置（用于距离计算）
      const WeightParams& params) {
      
      WeightedCircle result;
      result.center_x = calculated_center.x;
      result.center_y = calculated_center.y;
      result.fit_error = fit_error;
      result.point_count = points.size();
      
      // 计算到车辆的距离
      double dx = calculated_center.x - vehicle_position.x;
      double dy = calculated_center.y - vehicle_position.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      // 1. 计算各分量权重
      double W_count = std::min(static_cast<double>(points.size()) / 
                              params.ideal_point_count, 1.0);
      
      double W_error = std::exp(-fit_error / params.error_threshold);
      
      double W_distance = std::exp(-distance / params.distance_decay);
      
      double W_angle = calculateAngleDiversity(points, calculated_center);
      
      // 2. 组合最终权重
      result.weight_translation = W_count * W_error * W_distance;
      
      // 旋转权重更注重角度多样性和距离
      result.weight_rotation = W_angle * W_distance;
      
      // 确保权重在合理范围内
      result.weight_translation = std::clamp(result.weight_translation, 0.0, 1.0);
      result.weight_rotation = std::clamp(result.weight_rotation, 0.0, 1.0);
      
      return result;
  }
  // 轻量级EPS自适应
    double compute_fast_adaptive_eps(const std::vector<Point>& points, double base_eps) {
        if (points.size() < 10) return base_eps;
        
        // 快速计算点云密度
        Point min_pt = points[0], max_pt = points[0];
        for (const auto& p : points) {
            min_pt.x = std::min(min_pt.x, p.x);
            min_pt.y = std::min(min_pt.y, p.y);
            max_pt.x = std::max(max_pt.x, p.x);
            max_pt.y = std::max(max_pt.y, p.y);
        }
        
        double area = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y);
        double density = points.size() / (area + 1e-6);
        
        // 简单密度自适应
        double adaptive_factor = 1.0;
        if (density < 0.1) adaptive_factor = 1.3;  // 稀疏点云，增大EPS
        else if (density > 10.0) adaptive_factor = 0.7;  // 密集点云，减小EPS
        
        return base_eps * adaptive_factor;
    }

    // 预计算网格邻域关系
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> 
    precompute_grid_neighbors(const std::map<std::pair<int, int>, std::vector<int>>& grid) {
        std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> neighbors;
        
        for (const auto& [grid_coord, _] : grid) {
            auto [x, y] = grid_coord;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    neighbors[{x, y}].push_back({x + dx, y + dy});
                }
            }
        }
        return neighbors;
    }
  
	// 自适应 DBSCAN 聚类
	std::vector<int> adaptive_dbscan(const std::vector<Point>& points) {
		if (int(points.size()) < min_cluster_points) {
		    return std::vector<int>(points.size(), -1);
		}
		
		double base_eps = cluster_eps;
        
        // 1. 快速EPS自适应（避免复杂计算）
        double eps = compute_fast_adaptive_eps(points, base_eps);
        
        // 2. 保持网格划分，但优化搜索策略
        double grid_size = eps;
        std::map<std::pair<int, int>, std::vector<int>> grid;
        
        for (size_t i = 0; i < points.size(); ++i) {
            int grid_x = static_cast<int>(points[i].x / grid_size);
            int grid_y = static_cast<int>(points[i].y / grid_size);
            grid[{grid_x, grid_y}].push_back(i);
        }
        
        // 3. 优化：预计算网格邻域关系
        auto grid_neighbors = precompute_grid_neighbors(grid);
        
        UnionFind uf(points.size());
        std::vector<bool> is_core(points.size(), false);
        std::vector<int> neighbor_counts(points.size(), 0);
        
        // 4. 第一阶段：快速计数
        #pragma omp parallel for if(points.size() > 100)
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& current_point = points[i];
            int grid_x = static_cast<int>(current_point.x / grid_size);
            int grid_y = static_cast<int>(current_point.y / grid_size);
            
            int count = 0;
            // 只搜索相邻的9个网格
            for (const auto& neighbor_grid : grid_neighbors.at({grid_x, grid_y})) {
                auto it = grid.find(neighbor_grid);
                if (it != grid.end()) {
                    for (int idx : it->second) {
                        if ((current_point - points[idx]).squaredNorm() < eps * eps) {
                            count++;
                            if (count >= min_cluster_points) break;
                        }
                    }
                }
                if (count >= min_cluster_points) break;
            }
            neighbor_counts[i] = count;
        }
        
        // 5. 第二阶段：核心点判定和合并（优化合并策略）
        for (size_t i = 0; i < points.size(); ++i) {
            if (neighbor_counts[i] >= min_cluster_points) {
                is_core[i] = true;
                
                const auto& current_point = points[i];
                int grid_x = static_cast<int>(current_point.x / grid_size);
                int grid_y = static_cast<int>(current_point.y / grid_size);
                
                // 只与邻近的核心点合并，减少合并操作
                for (const auto& neighbor_grid : grid_neighbors.at({grid_x, grid_y})) {
                    auto it = grid.find(neighbor_grid);
                    if (it != grid.end()) {
                        for (int idx : it->second) {
                            if (is_core[idx] && 
                                (current_point - points[idx]).squaredNorm() < eps * eps) {
                                uf.unite(i, idx);
                            }
                        }
                    }
                }
            }
        }
        
        // 6. 快速标签分配（保持原有逻辑）
        std::vector<int> labels(points.size(), -1);
        std::map<int, int> cluster_map;
        int next_cluster_id = 0;
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (is_core[i]) {
                int root = uf.find(i);
                if (cluster_map.find(root) == cluster_map.end()) {
                    cluster_map[root] = next_cluster_id++;
                }
                labels[i] = cluster_map[root];
            }
        }
        
        // 7. 简化边界点分配：单次遍历
        for (size_t i = 0; i < points.size(); ++i) {
            if (!is_core[i] && labels[i] == -1) {
                int grid_x = static_cast<int>(points[i].x / grid_size);
                int grid_y = static_cast<int>(points[i].y / grid_size);
                
                for (const auto& neighbor_grid : grid_neighbors.at({grid_x, grid_y})) {
                    auto it = grid.find(neighbor_grid);
                    if (it != grid.end()) {
                        for (int idx : it->second) {
                            if (labels[idx] != -1 && 
                                (points[i] - points[idx]).squaredNorm() < eps * eps) {
                                labels[i] = labels[idx];
                                break;
                            }
                        }
                    }
                    if (labels[i] != -1) break;
                }
            }
        }
        
        return labels;
    }
};

} // namespace landmark_localization
