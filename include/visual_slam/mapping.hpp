/**
 * Local Mapping
 * Keyframe processing, triangulation, and local bundle adjustment
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include "visual_slam/frame.hpp"
#include "visual_slam/map.hpp"
#include "visual_slam/feature/orb_extractor.hpp"
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace visual_slam {

// Forward declaration
class LoopClosing;

/**
 * Local Mapping Configuration
 */
struct MappingConfig {
    // Triangulation
    float min_parallax = 1.0f;          // degrees
    float max_reproj_error = 2.0f;      // pixels

    // Bundle adjustment
    int local_ba_keyframes = 7;
    int local_ba_iterations = 10;

    // Culling
    float culling_redundancy = 0.9f;    // Remove KF if 90% points seen by others
    int min_observations = 3;           // Min observations for map point
};

/**
 * Local Mapping Thread
 * Processes new keyframes, creates map points, performs local BA
 */
class LocalMapping {
public:
    LocalMapping(MapPtr map, const MappingConfig& config);
    ~LocalMapping();

    /**
     * Insert new keyframe for processing
     */
    void insertKeyframe(KeyframePtr kf);

    /**
     * Main processing loop (run in separate thread)
     */
    void run();

    /**
     * Stop the mapping thread
     */
    void stop();

    /**
     * Request stop (non-blocking)
     */
    void requestStop();

    /**
     * Check if stopped
     */
    bool isStopped() const { return stopped_; }

    /**
     * Request reset
     */
    void requestReset();

    /**
     * Set loop closer for notification
     */
    void setLoopCloser(std::shared_ptr<LoopClosing> loop_closer);

    /**
     * Release new keyframes after loop closure
     */
    void release();

    /**
     * Check if busy (for tracking thread coordination)
     */
    bool isBusy() const { return processing_keyframe_; }

    /**
     * Check if accepting new keyframes
     */
    bool acceptKeyframes() const { return accept_keyframes_; }

    /**
     * Set accepting keyframes flag
     */
    void setAcceptKeyframes(bool flag) { accept_keyframes_ = flag; }

private:
    // Processing steps
    void processKeyframe();
    void createMapPoints();
    void triangulateNewMapPoints(KeyframePtr kf1, KeyframePtr kf2);
    void fuseMapPoints();
    void localBundleAdjustment();
    void keyframeCulling();
    void mapPointCulling();

    // Triangulation
    bool triangulatePoint(const Eigen::Vector3d& ray1,
                          const Eigen::Vector3d& ray2,
                          const SE3& T1,
                          const SE3& T2,
                          Eigen::Vector3d& point3d);

    bool checkTriangulation(const Eigen::Vector3d& point3d,
                           const SE3& pose1,
                           const SE3& pose2,
                           const Eigen::Vector2d& obs1,
                           const Eigen::Vector2d& obs2);

    MapPtr map_;
    MappingConfig config_;

    // Keyframe queue
    std::queue<KeyframePtr> keyframe_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    KeyframePtr current_keyframe_;
    std::vector<MapPointPtr> recent_map_points_;

    // Thread control
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> stopped_{false};
    std::atomic<bool> processing_keyframe_{false};
    std::atomic<bool> accept_keyframes_{true};
    std::atomic<bool> reset_requested_{false};

    // Connections
    std::weak_ptr<LoopClosing> loop_closer_;
    std::unique_ptr<FeatureMatcher> matcher_;
};

}  // namespace visual_slam
