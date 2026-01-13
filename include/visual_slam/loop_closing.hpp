/**
 * Loop Closing
 * Place recognition and pose graph optimization
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include "visual_slam/frame.hpp"
#include "visual_slam/map.hpp"
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace visual_slam {

// Forward declaration
class LocalMapping;

/**
 * Loop Closing Configuration
 */
struct LoopClosingConfig {
    // Detection
    float min_score = 0.05f;            // Min BoW score for candidate
    int min_consistency = 3;            // Min consecutive detections
    int min_inliers = 25;               // Min inliers for geometric verification

    // Verification
    float max_reproj_error = 3.0f;      // pixels
    int ransac_iterations = 300;

    // Optimization
    bool enable_pose_graph = true;
    int pose_graph_iterations = 20;
};

/**
 * Bag of Words Database for place recognition
 */
class BoWDatabase {
public:
    BoWDatabase();

    /**
     * Add keyframe to database
     */
    void add(KeyframePtr kf);

    /**
     * Remove keyframe from database
     */
    void erase(KeyframePtr kf);

    /**
     * Query similar keyframes
     * @param kf Query keyframe
     * @param min_score Minimum similarity score
     * @return Vector of (keyframe, score) pairs
     */
    std::vector<std::pair<KeyframePtr, float>> query(KeyframePtr kf, float min_score);

    /**
     * Clear database
     */
    void clear();

    /**
     * Load vocabulary from file
     */
    bool loadVocabulary(const std::string& filename);

private:
    // Inverted index: word_id -> (keyframe, weight)
    std::map<int, std::vector<std::pair<KeyframePtr, float>>> inverted_index_;

    // Vocabulary (DBoW2-style)
    cv::Ptr<cv::BOWImgDescriptorExtractor> bow_extractor_;
    cv::Mat vocabulary_;

    std::mutex mutex_;
};

/**
 * Loop Closing Thread
 * Detects loops and corrects drift via pose graph optimization
 */
class LoopClosing {
public:
    LoopClosing(MapPtr map, const LoopClosingConfig& config);
    ~LoopClosing();

    /**
     * Insert new keyframe for loop detection
     */
    void insertKeyframe(KeyframePtr kf);

    /**
     * Main processing loop
     */
    void run();

    /**
     * Stop the loop closing thread
     */
    void stop();

    /**
     * Request stop
     */
    void requestStop();

    /**
     * Check if stopped
     */
    bool isStopped() const { return stopped_; }

    /**
     * Set local mapper for coordination
     */
    void setLocalMapper(std::shared_ptr<LocalMapping> local_mapper);

    /**
     * Load vocabulary
     */
    bool loadVocabulary(const std::string& filename);

    /**
     * Get correction transformation for tracking thread
     */
    SE3 getCorrection() const;

    /**
     * Check if currently correcting
     */
    bool isRunningGlobalBA() const { return running_global_ba_; }

private:
    // Detection
    bool detectLoop();
    bool checkConsistency(KeyframePtr candidate);
    std::vector<KeyframePtr> detectCandidates(KeyframePtr kf);

    // Verification
    bool geometricVerification(KeyframePtr query, KeyframePtr candidate,
                               SE3& relative_pose,
                               std::vector<std::pair<int, int>>& inliers);

    // Correction
    void correctLoop();
    void searchAndFuse(KeyframePtr kf_target, const SE3& correction);
    void optimizeEssentialGraph();
    void runGlobalBundleAdjustment();

    MapPtr map_;
    LoopClosingConfig config_;

    // Database
    std::unique_ptr<BoWDatabase> bow_database_;

    // Keyframe queue
    std::queue<KeyframePtr> keyframe_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    KeyframePtr current_keyframe_;
    KeyframePtr matched_keyframe_;

    // Consistency check
    std::vector<KeyframePtr> consistent_candidates_;
    std::vector<std::vector<KeyframePtr>> consistency_groups_;

    // Loop correction
    SE3 loop_correction_;
    std::vector<LoopCandidate> detected_loops_;

    // Thread control
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> stopped_{false};
    std::atomic<bool> running_global_ba_{false};
    std::atomic<bool> finished_global_ba_{false};

    // Connections
    std::weak_ptr<LocalMapping> local_mapper_;
    std::unique_ptr<FeatureMatcher> matcher_;

    // Vocabulary
    std::string vocabulary_path_;
};

}  // namespace visual_slam
