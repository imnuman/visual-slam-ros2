/**
 * Frame and Keyframe Data Structures
 * Stores visual features and pose information
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include <opencv2/features2d.hpp>
#include <mutex>
#include <set>

namespace visual_slam {

/**
 * Frame - Single image with extracted features
 */
class Frame {
public:
    Frame() : id_(-1), timestamp_(0.0), is_keyframe_(false) {}

    Frame(int id, double timestamp, const cv::Mat& image_left,
          const cv::Mat& image_right = cv::Mat());

    // Getters
    int id() const { return id_; }
    double timestamp() const { return timestamp_; }
    const cv::Mat& imageLeft() const { return image_left_; }
    const cv::Mat& imageRight() const { return image_right_; }
    bool isStereo() const { return !image_right_.empty(); }

    const std::vector<cv::KeyPoint>& keypoints() const { return keypoints_; }
    const cv::Mat& descriptors() const { return descriptors_; }
    const std::vector<cv::KeyPoint>& keypointsRight() const { return keypoints_right_; }

    SE3 pose() const { return pose_; }
    void setPose(const SE3& pose) { pose_ = pose; }

    // Feature access
    int numFeatures() const { return keypoints_.size(); }
    cv::KeyPoint keypoint(int idx) const { return keypoints_[idx]; }
    cv::Mat descriptor(int idx) const { return descriptors_.row(idx); }
    Eigen::Vector3d bearing(int idx) const;

    // Map point associations
    MapPointPtr mapPoint(int idx) const;
    void setMapPoint(int idx, MapPointPtr mp);
    bool hasMapPoint(int idx) const;
    std::vector<MapPointPtr> mapPoints() const { return map_points_; }

    // Stereo matching
    float stereoDepth(int idx) const;
    Eigen::Vector3d unproject(int idx) const;

    // Grid-based feature access for efficient matching
    std::vector<int> getFeaturesInArea(float x, float y, float r) const;

    // Set features
    void setKeypoints(const std::vector<cv::KeyPoint>& kps,
                      const cv::Mat& desc,
                      const std::vector<cv::KeyPoint>& kps_right = {});

protected:
    int id_;
    double timestamp_;
    bool is_keyframe_;

    cv::Mat image_left_;
    cv::Mat image_right_;

    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keypoints_right_;
    std::vector<float> stereo_depths_;

    SE3 pose_;  // T_world_camera

    std::vector<MapPointPtr> map_points_;

    // Grid for efficient feature search
    static constexpr int GRID_COLS = 64;
    static constexpr int GRID_ROWS = 48;
    std::vector<std::vector<int>> grid_;

    void assignFeaturesToGrid();
};

/**
 * Keyframe - Frame selected for mapping and optimization
 */
class Keyframe : public Frame {
public:
    Keyframe() : Frame() { is_keyframe_ = true; }

    Keyframe(int id, double timestamp, const cv::Mat& image_left,
             const cv::Mat& image_right = cv::Mat());

    static KeyframePtr create(int id, double timestamp,
                              const cv::Mat& image_left,
                              const cv::Mat& image_right = cv::Mat());

    // Covisibility graph
    void addConnection(KeyframePtr kf, int weight);
    void eraseConnection(KeyframePtr kf);
    std::vector<KeyframePtr> getConnectedKeyframes() const;
    std::vector<KeyframePtr> getBestCovisibilityKeyframes(int n) const;
    int getWeight(KeyframePtr kf) const;

    // Spanning tree
    void setParent(KeyframePtr parent) { parent_ = parent; }
    KeyframePtr parent() const { return parent_; }
    void addChild(KeyframePtr child) { children_.insert(child); }
    void eraseChild(KeyframePtr child) { children_.erase(child); }
    std::set<KeyframePtr> children() const { return children_; }

    // Loop closure
    void addLoopEdge(KeyframePtr kf) { loop_edges_.insert(kf); }
    std::set<KeyframePtr> loopEdges() const { return loop_edges_; }

    // Bad flag
    void setBad() { is_bad_ = true; }
    bool isBad() const { return is_bad_; }

    // Vocabulary for place recognition
    void computeBoW(cv::Ptr<cv::BOWImgDescriptorExtractor> bow_extractor);
    const cv::Mat& bowVector() const { return bow_vector_; }

private:
    mutable std::mutex connections_mutex_;
    std::map<KeyframePtr, int> connected_keyframes_;

    KeyframePtr parent_;
    std::set<KeyframePtr> children_;
    std::set<KeyframePtr> loop_edges_;

    bool is_bad_ = false;

    cv::Mat bow_vector_;
};

}  // namespace visual_slam
