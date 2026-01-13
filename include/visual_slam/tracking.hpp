/**
 * Visual Tracking
 * Frame-to-frame tracking and localization
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include "visual_slam/frame.hpp"
#include "visual_slam/map.hpp"
#include "visual_slam/feature/orb_extractor.hpp"
#include "visual_slam/imu/preintegration.hpp"

namespace visual_slam {

// Forward declarations
class LocalMapping;
class LoopClosing;

/**
 * Tracking Configuration
 */
struct TrackingConfig {
    // Feature detection
    int num_features = 2000;
    float scale_factor = 1.2f;
    int num_levels = 8;

    // Tracking thresholds
    int min_inliers = 15;
    int min_matches = 20;
    float reproj_threshold = 2.0f;

    // Keyframe creation
    int keyframe_interval = 5;
    float keyframe_translation = 0.1f;  // meters
    float keyframe_rotation = 5.0f;     // degrees
    int min_tracked_ratio = 0.5;

    // IMU
    bool use_imu = true;

    // Motion model
    bool use_motion_model = true;
};

/**
 * Visual Tracking Thread
 * Processes incoming frames and estimates camera pose
 */
class Tracking {
public:
    Tracking(MapPtr map, const TrackingConfig& config,
             const CameraIntrinsics& camera,
             const IMUParams& imu_params = IMUParams());

    /**
     * Process new frame
     * @return Current pose estimate
     */
    SE3 track(const cv::Mat& image_left,
              const cv::Mat& image_right,
              double timestamp,
              const std::vector<IMUMeasurement>& imu = {});

    // Thread control
    void setLocalMapper(std::shared_ptr<LocalMapping> local_mapper);
    void setLoopCloser(std::shared_ptr<LoopClosing> loop_closer);

    // State
    TrackingState state() const { return state_; }
    FramePtr currentFrame() const { return current_frame_; }
    KeyframePtr referenceKeyframe() const { return reference_kf_; }

    // Relocalization
    bool relocalize(FramePtr frame);

    // Reset
    void reset();

private:
    // Processing stages
    void extractFeatures(FramePtr frame);
    bool initialize();
    bool trackWithMotionModel();
    bool trackReferenceKeyframe();
    bool trackLocalMap();

    // Keyframe management
    bool needNewKeyframe();
    void createKeyframe();
    void updateLocalMap();
    void updateLocalKeyframes();
    void updateLocalMapPoints();

    // Pose estimation
    bool estimatePoseEpnP(FramePtr frame, std::vector<MapPointPtr>& map_points);
    bool estimatePoseEssential(FramePtr prev_frame, FramePtr curr_frame);
    int optimizePose(FramePtr frame);

    // IMU integration
    void integrateIMU(double t_start, double t_end);
    SE3 predictPoseIMU();

    // Culling
    void cullMapPoints();

    MapPtr map_;
    TrackingConfig config_;
    CameraIntrinsics camera_;
    IMUParams imu_params_;

    std::unique_ptr<ORBExtractor> orb_extractor_;
    std::unique_ptr<FeatureMatcher> matcher_;
    std::unique_ptr<IMUPreintegration> imu_preint_;
    std::unique_ptr<VIOInitializer> vio_initializer_;

    TrackingState state_ = TrackingState::NOT_INITIALIZED;

    FramePtr current_frame_;
    FramePtr last_frame_;
    KeyframePtr reference_kf_;
    KeyframePtr last_keyframe_;

    // Local map
    std::vector<KeyframePtr> local_keyframes_;
    std::vector<MapPointPtr> local_map_points_;

    // Motion model
    SE3 velocity_;
    bool has_velocity_ = false;

    // IMU state
    IMUPreintegration::State imu_state_;
    IMUBuffer imu_buffer_;

    // Frame counter
    int frame_id_ = 0;
    int frames_since_keyframe_ = 0;

    // Thread connections
    std::weak_ptr<LocalMapping> local_mapper_;
    std::weak_ptr<LoopClosing> loop_closer_;
};

}  // namespace visual_slam
