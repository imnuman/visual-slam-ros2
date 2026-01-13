/**
 * Map Point - 3D landmark in the map
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include <mutex>
#include <map>

namespace visual_slam {

/**
 * MapPoint - 3D point in the world frame
 */
class MapPoint {
public:
    MapPoint() : id_(-1), is_bad_(false), visible_count_(0), found_count_(0) {}

    MapPoint(int id, const Eigen::Vector3d& pos, KeyframePtr ref_kf);

    static MapPointPtr create(int id, const Eigen::Vector3d& pos, KeyframePtr ref_kf);

    // Getters
    int id() const { return id_; }
    Eigen::Vector3d position() const {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        return position_;
    }

    void setPosition(const Eigen::Vector3d& pos) {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        position_ = pos;
    }

    // Reference keyframe
    KeyframePtr referenceKeyframe() const { return reference_kf_; }
    void setReferenceKeyframe(KeyframePtr kf) { reference_kf_ = kf; }

    // Observations
    void addObservation(KeyframePtr kf, int idx);
    void eraseObservation(KeyframePtr kf);
    std::map<KeyframePtr, int> observations() const {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        return observations_;
    }
    int numObservations() const {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        return observations_.size();
    }

    int getIndexInKeyframe(KeyframePtr kf) const;
    bool isInKeyframe(KeyframePtr kf) const;

    // Descriptor
    cv::Mat descriptor() const { return descriptor_; }
    void computeDistinctiveDescriptor();

    // Normal and depth
    Eigen::Vector3d normal() const { return normal_; }
    void updateNormalAndDepth();
    float minDistance() const { return min_distance_; }
    float maxDistance() const { return max_distance_; }

    // Tracking statistics
    void increaseVisible(int n = 1) { visible_count_ += n; }
    void increaseFound(int n = 1) { found_count_ += n; }
    float foundRatio() const {
        return visible_count_ > 0 ? (float)found_count_ / visible_count_ : 0.0f;
    }

    // Quality
    MapPointQuality quality() const;
    bool isBad() const { return is_bad_; }
    void setBad();

    // Predict scale level for a given distance
    int predictScale(float dist, float log_scale_factor) const;

private:
    int id_;
    Eigen::Vector3d position_;
    Eigen::Vector3d normal_;

    KeyframePtr reference_kf_;
    std::map<KeyframePtr, int> observations_;

    cv::Mat descriptor_;

    float min_distance_ = 0.0f;
    float max_distance_ = 0.0f;

    int visible_count_;
    int found_count_;

    bool is_bad_;

    mutable std::mutex pos_mutex_;
    mutable std::mutex obs_mutex_;
};

}  // namespace visual_slam
