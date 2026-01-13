/**
 * Visual SLAM Type Definitions
 * Common data structures used throughout the system
 *
 * Author: Al Numan
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <map>

namespace visual_slam {

// Forward declarations
class Frame;
class Keyframe;
class MapPoint;
class Map;

using FramePtr = std::shared_ptr<Frame>;
using KeyframePtr = std::shared_ptr<Keyframe>;
using MapPointPtr = std::shared_ptr<MapPoint>;
using MapPtr = std::shared_ptr<Map>;

/**
 * SE3 Transformation (rigid body motion)
 */
class SE3 {
public:
    SE3() : rotation_(Eigen::Quaterniond::Identity()), translation_(Eigen::Vector3d::Zero()) {}

    SE3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t)
        : rotation_(q.normalized()), translation_(t) {}

    SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
        : rotation_(R), translation_(t) { rotation_.normalize(); }

    explicit SE3(const Eigen::Matrix4d& T)
        : rotation_(T.block<3, 3>(0, 0)), translation_(T.block<3, 1>(0, 3)) {
        rotation_.normalize();
    }

    // Getters
    Eigen::Quaterniond rotation() const { return rotation_; }
    Eigen::Matrix3d rotationMatrix() const { return rotation_.toRotationMatrix(); }
    Eigen::Vector3d translation() const { return translation_; }

    Eigen::Matrix4d matrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation_.toRotationMatrix();
        T.block<3, 1>(0, 3) = translation_;
        return T;
    }

    // Operations
    SE3 inverse() const {
        Eigen::Quaterniond q_inv = rotation_.inverse();
        return SE3(q_inv, -(q_inv * translation_));
    }

    SE3 operator*(const SE3& other) const {
        return SE3(rotation_ * other.rotation_,
                   rotation_ * other.translation_ + translation_);
    }

    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const {
        return rotation_ * p + translation_;
    }

    // Lie algebra
    static SE3 exp(const Eigen::Matrix<double, 6, 1>& xi);
    Eigen::Matrix<double, 6, 1> log() const;

private:
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d translation_;
};

/**
 * Camera intrinsic parameters
 */
struct CameraIntrinsics {
    double fx, fy;      // Focal length
    double cx, cy;      // Principal point
    double k1, k2, p1, p2, k3;  // Distortion coefficients
    int width, height;

    cv::Mat K() const {
        return (cv::Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);
    }

    cv::Mat distCoeffs() const {
        return (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
    }

    Eigen::Matrix3d K_eigen() const {
        Eigen::Matrix3d K;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        return K;
    }
};

/**
 * Stereo camera parameters
 */
struct StereoParams {
    CameraIntrinsics left;
    CameraIntrinsics right;
    double baseline;        // Baseline in meters
    SE3 T_left_right;       // Transform from left to right camera

    double bf() const { return baseline * left.fx; }
};

/**
 * IMU noise parameters
 */
struct IMUParams {
    double gyro_noise;          // rad/s/sqrt(Hz)
    double accel_noise;         // m/s^2/sqrt(Hz)
    double gyro_bias_noise;     // rad/s^2/sqrt(Hz)
    double accel_bias_noise;    // m/s^3/sqrt(Hz)
    double rate;                // Hz
    SE3 T_cam_imu;              // Transform from camera to IMU
};

/**
 * IMU measurement
 */
struct IMUMeasurement {
    double timestamp;
    Eigen::Vector3d gyro;       // Angular velocity (rad/s)
    Eigen::Vector3d accel;      // Linear acceleration (m/s^2)
};

/**
 * Feature observation
 */
struct Observation {
    int keyframe_id;
    int keypoint_idx;
    Eigen::Vector2d uv;         // Image coordinates
    double depth;               // Depth (if available, -1 otherwise)
    bool is_stereo;
    Eigen::Vector2d uv_right;   // Right image coordinates (for stereo)
};

/**
 * Tracking state
 */
enum class TrackingState {
    NOT_INITIALIZED,
    INITIALIZING,
    TRACKING,
    LOST,
    RELOCALIZATION
};

/**
 * Map point quality
 */
enum class MapPointQuality {
    GOOD,
    MARGINAL,
    BAD
};

/**
 * Loop closure candidate
 */
struct LoopCandidate {
    int query_kf_id;
    int match_kf_id;
    double score;
    SE3 relative_pose;
    std::vector<std::pair<int, int>> inlier_matches;
};

/**
 * Optimization result
 */
struct OptimizationResult {
    bool converged;
    int iterations;
    double initial_error;
    double final_error;
    double time_ms;
};

}  // namespace visual_slam
