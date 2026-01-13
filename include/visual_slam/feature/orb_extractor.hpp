/**
 * ORB Feature Extractor
 * Multi-scale ORB feature detection with uniform distribution
 *
 * Author: Al Numan
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace visual_slam {

/**
 * ORB Feature Extractor with image pyramid
 */
class ORBExtractor {
public:
    struct Config {
        int num_features = 2000;
        float scale_factor = 1.2f;
        int num_levels = 8;
        int ini_th_fast = 20;
        int min_th_fast = 7;

        // Grid parameters for uniform distribution
        int grid_rows = 30;
        int grid_cols = 40;
    };

    explicit ORBExtractor(const Config& config = Config());

    /**
     * Extract ORB features from image
     *
     * @param image Input grayscale image
     * @param keypoints Output keypoints
     * @param descriptors Output descriptors
     */
    void detect(const cv::Mat& image,
                std::vector<cv::KeyPoint>& keypoints,
                cv::Mat& descriptors);

    /**
     * Detect and compute in stereo pair
     */
    void detectStereo(const cv::Mat& image_left,
                      const cv::Mat& image_right,
                      std::vector<cv::KeyPoint>& keypoints_left,
                      std::vector<cv::KeyPoint>& keypoints_right,
                      cv::Mat& descriptors_left,
                      cv::Mat& descriptors_right,
                      std::vector<float>& stereo_depths,
                      float baseline, float fx);

    // Getters
    int numLevels() const { return config_.num_levels; }
    float scaleFactor() const { return config_.scale_factor; }
    std::vector<float> scaleFactors() const { return scale_factors_; }
    std::vector<float> invScaleFactors() const { return inv_scale_factors_; }
    std::vector<float> sigmaSq() const { return level_sigma2_; }
    std::vector<float> invSigmaSq() const { return inv_level_sigma2_; }

    const std::vector<cv::Mat>& imagePyramid() const { return image_pyramid_; }

private:
    void computePyramid(const cv::Mat& image);
    void computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>>& all_keypoints);
    std::vector<cv::KeyPoint> distributeOctTree(const std::vector<cv::KeyPoint>& keypoints,
                                                 int minX, int maxX, int minY, int maxY,
                                                 int num_features);
    void computeDescriptors(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::Mat& descriptors);
    void computeOrientation(const cv::Mat& image,
                           std::vector<cv::KeyPoint>& keypoints);

    float stereoMatch(const cv::KeyPoint& kp_left,
                      const cv::Mat& desc_left,
                      const std::vector<cv::KeyPoint>& keypoints_right,
                      const cv::Mat& descriptors_right,
                      int level);

    Config config_;

    std::vector<cv::Mat> image_pyramid_;
    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    std::vector<float> level_sigma2_;
    std::vector<float> inv_level_sigma2_;

    std::vector<int> features_per_level_;
    std::vector<cv::Point> pattern_;
    std::vector<int> umax_;  // For orientation computation
};

/**
 * Feature Matcher using brute-force with ratio test
 */
class FeatureMatcher {
public:
    struct Config {
        float ratio_threshold = 0.75f;
        int hamming_threshold = 50;
        bool check_orientation = true;
        float orientation_threshold = 30.0f;  // degrees
    };

    explicit FeatureMatcher(const Config& config = Config());

    /**
     * Match features between two frames
     */
    std::vector<cv::DMatch> match(const cv::Mat& desc1, const cv::Mat& desc2);

    /**
     * Match frame features to map points using projection
     */
    int searchByProjection(Frame& frame,
                          const std::vector<MapPointPtr>& map_points,
                          float radius_threshold = 3.0f);

    /**
     * Match for initialization (wider search)
     */
    std::vector<cv::DMatch> matchForInitialization(
        const std::vector<cv::KeyPoint>& kps1,
        const std::vector<cv::KeyPoint>& kps2,
        const cv::Mat& desc1,
        const cv::Mat& desc2);

    /**
     * Match for triangulation between keyframes
     */
    int searchForTriangulation(KeyframePtr kf1, KeyframePtr kf2,
                               std::vector<std::pair<int, int>>& matches);

    /**
     * Match for loop closure
     */
    int searchByBoW(KeyframePtr kf1, KeyframePtr kf2,
                    std::vector<std::pair<int, int>>& matches);

    /**
     * Compute descriptor distance
     */
    static int descriptorDistance(const cv::Mat& a, const cv::Mat& b);

private:
    Config config_;
    cv::Ptr<cv::BFMatcher> matcher_;
};

}  // namespace visual_slam
