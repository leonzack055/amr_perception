#pragma once

#include <vector>
#include <deque>
#include <memory>
#include <cmath>
#include <algorithm>

#include "amr_reflector_noise_handling/types.hpp"
#include "amr_reflector_noise_handling/types/reflector_common.hpp"

namespace amr_reflector_noise_handling {

/**
 * @brief Global Reflector Tracker
 * 
 * This class manages global IDs for reflectors, filters noise detections,
 * and maintains position consistency through EMA filtering and uncertainty estimation.
 * 
 * Key features:
 * - ID persistence: Confirmed reflector IDs are permanent
 * - Noise filtering: Sliding window-based continuity detection
 * - Position smoothing: EMA filtering with uncertainty estimation
 * - Fast re-matching: INACTIVE reflectors still participate in matching
 * - Confidence calculation: Combines detection quality, uncertainty, and continuity
 */
class GlobalReflectorTracker {
public:
    /**
     * @brief Configuration parameters for the tracker
     */
    struct Config {
        // === Matching Parameters ===
        double match_distance_threshold;    // Matching distance threshold for CONFIRMED/TENTATIVE (m)
        double match_distance_inactive;     // Matching distance threshold for INACTIVE state (m)
        
        // === Continuity Detection Parameters ===
        double confirm_time_window;         // Time window for confirmation (s)
        int min_detections_in_window;       // Minimum detections in window to confirm
        
        // === State Management Parameters ===
        double inactive_timeout;            // Timeout to enter INACTIVE state (s)
        double max_inactive_time;          // Maximum time to keep INACTIVE reflectors (s)
        
        // === Position Filtering Parameters ===
        double position_filter_alpha;       // EMA filter coefficient (0-1)
        double position_filter_beta;        // Variance filter coefficient (0-1)
        double min_std_dev;                 // Minimum standard deviation (m)
        double max_std_dev;                 // Maximum standard deviation (m)
        
        // === Confidence Parameters ===
        double min_confidence_to_track;     // Minimum confidence to track
        double confidence_filter_alpha;     // Confidence EMA filter coefficient
        
        // === Diameter Filtering Parameters ===
        double diameter_filter_alpha;       // Diameter EMA filter coefficient
        
        /**
         * @brief Default constructor with sensible defaults
         */
        Config()
            : match_distance_threshold(0.3)
            , match_distance_inactive(0.5)
            , confirm_time_window(1.0)
            , min_detections_in_window(6)
            , inactive_timeout(5.0)
            , max_inactive_time(60.0)
            , position_filter_alpha(0.3)
            , position_filter_beta(0.2)
            , min_std_dev(0.02)
            , max_std_dev(0.5)
            , min_confidence_to_track(0.3)
            , confidence_filter_alpha(0.2)
            , diameter_filter_alpha(0.3)
        {}
    };
    
    /**
     * @brief Match result from findBestMatch
     */
    struct MatchResult {
        int tracker_id;                     // Matched tracker index (-1 if no match)
        double distance;                    // Matching distance
    };
    
    /**
     * @brief Constructor
     * @param config Configuration parameters
     */
    explicit GlobalReflectorTracker(const Config& config = Config());
    
    /**
     * @brief Destructor
     */
    ~GlobalReflectorTracker();
    
    /**
     * @brief Update tracker with new detections
     * 
     * This is the main function that should be called for each frame.
     * It performs data association, updates position filters, and manages state transitions.
     * 
     * @param detected_reflectors Reflectors detected in current frame (in laser frame)
     * @param laser_to_world Transform from laser frame to world frame
     * @param timestamp Current timestamp in seconds ns
     */
    void update(const std::vector<DetectedReflector> &detected_reflectors,
                const transforms::Rigid3d &laser_to_world, int64_t timestamp);

    /**
     * @brief Get confirmed reflectors (CONFIRMED state)
     * 
     * These are reflectors that have passed the continuity check
     * and have permanent global IDs.
     * 
     * @return Vector of confirmed tracked reflectors
     */
    std::vector<TrackedReflector> getConfirmedReflectors() const;
    
    /**
     * @brief Get all active reflectors (CONFIRMED + TENTATIVE)
     * 
     * This includes both confirmed and tentative reflectors that are
     * currently being tracked.
     * 
     * @return Vector of active tracked reflectors
     */
    std::vector<TrackedReflector> getAllActiveReflectors() const;
    
    /**
     * @brief Get all tracked reflectors (including INACTIVE)
     * 
     * This includes all reflectors that have ever been detected,
     * even those currently in INACTIVE state.
     * 
     * @return Vector of all tracked reflectors
     */
    std::vector<TrackedReflector> getAllTrackedReflectors() const;
    
    /**
     * @brief Reset the tracker
     * 
     * Clears all tracked reflectors and resets the next global ID counter.
     */
    void reset();
    
    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    const Config& getConfig() const { return config_; }
    
    /**
     * @brief Set new configuration
     * @param config New configuration
     */
    void setConfig(const Config& config) { config_ = config; }
    
    /**
     * @brief Get number of tracked reflectors
     * @return Number of tracked reflectors
     */
    size_t getTrackedReflectorCount() const { return tracked_reflectors_.size(); }
    
    /**
     * @brief Get number of confirmed reflectors
     * @return Number of confirmed reflectors
     */
    size_t getConfirmedReflectorCount() const;
    
private:
    /**
     * @brief Find best matching tracker for a global position
     * 
     * @param global_position Position in world frame
     * @param tracked_reflectors List of tracked reflectors to search
     * @param include_inactive Whether to include INACTIVE reflectors in matching
     * @return Match result with tracker ID and distance
     */
    MatchResult findBestMatch(
        const Point& global_position,
        const std::vector<TrackedReflector>& tracked_reflectors,
        bool include_inactive = true
    ) const;
    
    /**
     * @brief Update an existing tracked reflector with new detection
     * 
     * @param tracker Tracked reflector to update
     * @param detected New detection data
     * @param global_position Global position of the detection
     * @param timestamp Current timestamp ns
     */
    void updateTrackedReflector(TrackedReflector &tracker,
                                const DetectedReflector &detected,
                                const Point &global_position,
                                int64_t timestamp);

    /**
     * @brief Create a new tracked reflector from detection
     * 
     * @param detected Detection data
     * @param global_position Global position
     * @param global_id Global ID to assign
     * @param timestamp Current timestamp ns
     * @return New tracked reflector
     */
    TrackedReflector createTrackedReflector(const DetectedReflector &detected,
                                            const Point &global_position,
                                            int global_id, int64_t timestamp);

    /**
     * @brief Update position filter (EMA)
     * 
     * @param tracker Tracked reflector to update
     * @param measured_position Measured position
     */
    void updatePositionFilter(
        TrackedReflector& tracker,
        const Point& measured_position
    );
    
    /**
     * @brief Update position uncertainty estimation
     * 
     * @param tracker Tracked reflector to update
     * @param measured_position Measured position
     */
    void updatePositionUncertainty(
        TrackedReflector& tracker,
        const Point& measured_position
    );
    
    /**
     * @brief Update confidence estimation
     * 
     * @param tracker Tracked reflector to update
     * @param detection_confidence Confidence of the detection
     */
    void updateConfidence(
        TrackedReflector& tracker,
        double detection_confidence
    );
    
    /**
     * @brief Check continuity using sliding window
     *
     * @param tracker Tracked reflector to check
     * @return True if continuity condition is met
     */
    bool checkContinuity(TrackedReflector& tracker) const;
    
    /**
     * @brief Update state machine for a tracked reflector
     * 
     * @param tracker Tracked reflector to update
     * @param current_time Current timestamp ns
     */
    void updateState(TrackedReflector &tracker, int64_t current_time);

    /**
     * @brief Cleanup expired reflectors
     * 
     * Removes TENTATIVE reflectors that have not been confirmed
     * and INACTIVE reflectors that have exceeded max_inactive_time.
     * 
     * @param current_time Current timestamp ns
     */
    void cleanupExpiredReflectors(int64_t current_time);

    /**
     * @brief Transform point from laser frame to world frame
     * 
     * @param laser_point Point in laser frame
     * @param laser_to_world Transform from laser to world
     * @return Point in world frame
     */
    Point transformToGlobal(
        const Point& laser_point,
        const transforms::Rigid3d& laser_to_world
    ) const;
    
    // === Member Variables ===
    Config config_;                           // Configuration parameters
    std::vector<TrackedReflector> tracked_reflectors_;  // Tracked reflectors
    int next_global_id_;                       // Next global ID to assign
    int64_t current_timestamp_;                // Current timestamp ns
};

} // namespace amr_reflector_noise_handling