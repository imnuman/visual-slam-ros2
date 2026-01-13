/**
 * Bundle Adjustment
 * Non-linear optimization for camera poses and 3D points
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include "visual_slam/frame.hpp"
#include "visual_slam/map.hpp"
#include "visual_slam/imu/preintegration.hpp"

namespace visual_slam {

/**
 * Bundle Adjustment Configuration
 */
struct BAConfig {
    int max_iterations = 10;
    double function_tolerance = 1e-6;
    double parameter_tolerance = 1e-8;
    bool verbose = false;

    // Robust kernel
    bool use_huber = true;
    double huber_delta = 1.0;

    // Fixed parameters
    bool fix_scale = false;
    bool fix_first_pose = true;
};

/**
 * Bundle Adjustment Results
 */
struct BAResult {
    bool converged;
    int iterations;
    double initial_cost;
    double final_cost;
    double time_ms;
    int num_bad_observations;
};

/**
 * Local Bundle Adjustment
 * Optimizes poses of local keyframes and their observed map points
 */
class LocalBundleAdjustment {
public:
    LocalBundleAdjustment(const BAConfig& config = BAConfig());

    /**
     * Run local BA on keyframes
     * @param keyframes Keyframes to optimize (first may be fixed)
     * @param map_points Map points observed by keyframes
     * @param camera Camera intrinsics
     * @return Optimization result
     */
    BAResult optimize(std::vector<KeyframePtr>& keyframes,
                      std::vector<MapPointPtr>& map_points,
                      const CameraIntrinsics& camera);

    /**
     * Run visual-inertial local BA
     */
    BAResult optimizeVI(std::vector<KeyframePtr>& keyframes,
                        std::vector<MapPointPtr>& map_points,
                        const std::vector<IMUPreintegration::PreintegratedMeasurement>& imu_factors,
                        const CameraIntrinsics& camera);

private:
    BAConfig config_;
};

/**
 * Global Bundle Adjustment
 * Optimizes all keyframes and map points in the map
 */
class GlobalBundleAdjustment {
public:
    GlobalBundleAdjustment(const BAConfig& config = BAConfig());

    /**
     * Run global BA on entire map
     */
    BAResult optimize(MapPtr map, const CameraIntrinsics& camera);

    /**
     * Run global BA with IMU factors
     */
    BAResult optimizeVI(MapPtr map,
                        const std::vector<IMUPreintegration::PreintegratedMeasurement>& imu_factors,
                        const CameraIntrinsics& camera);

private:
    BAConfig config_;
};

/**
 * Pose-only optimization
 * Optimize single frame pose given map point observations
 */
class PoseOptimization {
public:
    PoseOptimization(const CameraIntrinsics& camera);

    /**
     * Optimize frame pose
     * @param frame Frame to optimize
     * @return Number of outliers
     */
    int optimize(FramePtr frame);

    /**
     * Optimize pose with IMU prior
     */
    int optimizeVI(FramePtr frame,
                   const IMUPreintegration::State& imu_prior,
                   const Eigen::Matrix<double, 15, 15>& imu_covariance);

private:
    CameraIntrinsics camera_;
    double chi2_threshold_ = 5.991;  // 95% confidence for 2 DOF
};

/**
 * Essential Graph Optimization
 * Pose graph optimization for loop closure
 */
class EssentialGraphOptimization {
public:
    struct Edge {
        int from_id;
        int to_id;
        SE3 measurement;
        Eigen::Matrix<double, 6, 6> information;
    };

    EssentialGraphOptimization(const BAConfig& config = BAConfig());

    /**
     * Optimize essential graph
     * @param keyframes All keyframes
     * @param loop_edges Loop closure edges
     * @return Optimization result
     */
    BAResult optimize(std::vector<KeyframePtr>& keyframes,
                      const std::vector<Edge>& loop_edges);

private:
    BAConfig config_;
};

/**
 * Sim3 Optimization for scale correction
 */
class Sim3Optimization {
public:
    struct Sim3Transform {
        double scale;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d translation;

        Eigen::Matrix4d matrix() const;
        Sim3Transform inverse() const;
        Eigen::Vector3d transform(const Eigen::Vector3d& p) const;
    };

    /**
     * Compute Sim3 transformation between two sets of 3D points
     */
    static Sim3Transform computeSim3(const std::vector<Eigen::Vector3d>& pts1,
                                     const std::vector<Eigen::Vector3d>& pts2);

    /**
     * Optimize Sim3 given correspondences
     */
    static Sim3Transform optimizeSim3(KeyframePtr kf1, KeyframePtr kf2,
                                      const std::vector<std::pair<int, int>>& matches);
};

}  // namespace visual_slam
