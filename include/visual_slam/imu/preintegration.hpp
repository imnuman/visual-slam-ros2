/**
 * IMU Preintegration
 * On-manifold preintegration for visual-inertial fusion
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include <deque>

namespace visual_slam {

/**
 * IMU Preintegrated Measurements
 * Based on Forster et al. "On-Manifold Preintegration for Real-Time
 * Visual-Inertial Odometry"
 */
class IMUPreintegration {
public:
    /**
     * IMU State (position, velocity, orientation, biases)
     */
    struct State {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();

        Eigen::Matrix<double, 15, 1> toVector() const;
        static State fromVector(const Eigen::Matrix<double, 15, 1>& v);
    };

    /**
     * Preintegrated measurement
     */
    struct PreintegratedMeasurement {
        Eigen::Vector3d delta_p;        // Position increment
        Eigen::Vector3d delta_v;        // Velocity increment
        Eigen::Quaterniond delta_q;     // Rotation increment
        double delta_t;                 // Time interval

        // Jacobians w.r.t. bias
        Eigen::Matrix3d J_p_bg;         // d(delta_p)/d(bias_gyro)
        Eigen::Matrix3d J_p_ba;         // d(delta_p)/d(bias_accel)
        Eigen::Matrix3d J_v_bg;
        Eigen::Matrix3d J_v_ba;
        Eigen::Matrix3d J_q_bg;

        // Covariance
        Eigen::Matrix<double, 9, 9> covariance;
    };

    IMUPreintegration(const IMUParams& params);

    /**
     * Reset preintegration
     */
    void reset();

    /**
     * Integrate single IMU measurement
     */
    void integrate(const IMUMeasurement& imu, double dt);

    /**
     * Integrate batch of IMU measurements
     */
    void integrateBatch(const std::vector<IMUMeasurement>& measurements);

    /**
     * Get preintegrated measurement
     */
    PreintegratedMeasurement getPreintegrated() const { return preintegrated_; }

    /**
     * Predict state using preintegrated measurement
     */
    State predict(const State& state) const;

    /**
     * Compute residual for optimization
     */
    Eigen::Matrix<double, 15, 1> computeResidual(const State& state_i,
                                                  const State& state_j) const;

    /**
     * Re-propagate with updated biases (for optimization)
     */
    void repropagate(const Eigen::Vector3d& bias_gyro,
                     const Eigen::Vector3d& bias_accel);

    // Getters
    double deltaT() const { return preintegrated_.delta_t; }
    Eigen::Vector3d deltaP() const { return preintegrated_.delta_p; }
    Eigen::Vector3d deltaV() const { return preintegrated_.delta_v; }
    Eigen::Quaterniond deltaQ() const { return preintegrated_.delta_q; }

private:
    IMUParams params_;
    PreintegratedMeasurement preintegrated_;

    // Noise matrices
    Eigen::Matrix<double, 6, 6> noise_cov_;

    // Store raw measurements for re-propagation
    std::vector<IMUMeasurement> measurements_;

    // Current biases (for linearization point)
    Eigen::Vector3d bias_gyro_lin_;
    Eigen::Vector3d bias_accel_lin_;

    static constexpr double GRAVITY = 9.81;
};

/**
 * IMU Buffer for managing incoming measurements
 */
class IMUBuffer {
public:
    IMUBuffer(size_t max_size = 1000) : max_size_(max_size) {}

    void addMeasurement(const IMUMeasurement& imu);
    void clear() { buffer_.clear(); }

    /**
     * Get measurements between two timestamps
     */
    std::vector<IMUMeasurement> getMeasurements(double t_start, double t_end) const;

    /**
     * Get latest measurement
     */
    IMUMeasurement latest() const;

    /**
     * Interpolate IMU measurement at specific time
     */
    IMUMeasurement interpolate(double t) const;

    size_t size() const { return buffer_.size(); }
    bool empty() const { return buffer_.empty(); }

private:
    std::deque<IMUMeasurement> buffer_;
    size_t max_size_;
    mutable std::mutex mutex_;
};

/**
 * Visual-Inertial Initializer
 * Initialize VIO from scratch using gyroscope integration and visual SfM
 */
class VIOInitializer {
public:
    struct Result {
        bool success;
        State initial_state;
        double scale;
        Eigen::Vector3d gravity;
        std::vector<KeyframePtr> keyframes;
        std::vector<MapPointPtr> map_points;
    };

    VIOInitializer(const IMUParams& params, const CameraIntrinsics& camera);

    /**
     * Add frame with IMU measurements
     */
    void addFrame(FramePtr frame, const std::vector<IMUMeasurement>& imu);

    /**
     * Try to initialize
     */
    Result tryInitialize();

    /**
     * Check if ready for initialization
     */
    bool isReady() const { return frames_.size() >= min_frames_; }

    void reset();

private:
    bool estimateGravityAndScale(std::vector<KeyframePtr>& keyframes,
                                 Eigen::Vector3d& gravity,
                                 double& scale);

    bool refineGravity(std::vector<KeyframePtr>& keyframes,
                       Eigen::Vector3d& gravity);

    IMUParams params_;
    CameraIntrinsics camera_;

    std::vector<FramePtr> frames_;
    std::vector<std::vector<IMUMeasurement>> imu_batches_;

    static constexpr int min_frames_ = 10;
    static constexpr double min_parallax_ = 30.0;  // pixels
};

}  // namespace visual_slam
