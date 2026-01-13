/**
 * Map - Contains keyframes and map points
 *
 * Author: Al Numan
 */

#pragma once

#include "visual_slam/types.hpp"
#include "visual_slam/frame.hpp"
#include "visual_slam/map_point.hpp"
#include <mutex>
#include <set>

namespace visual_slam {

/**
 * Map - Global map containing all keyframes and map points
 */
class Map {
public:
    Map() : max_kf_id_(0), max_mp_id_(0) {}

    // Keyframe operations
    void addKeyframe(KeyframePtr kf);
    void eraseKeyframe(KeyframePtr kf);
    std::vector<KeyframePtr> getAllKeyframes() const;
    KeyframePtr getKeyframe(int id) const;
    int numKeyframes() const;

    // Map point operations
    void addMapPoint(MapPointPtr mp);
    void eraseMapPoint(MapPointPtr mp);
    std::vector<MapPointPtr> getAllMapPoints() const;
    MapPointPtr getMapPoint(int id) const;
    int numMapPoints() const;

    // Reference map points (for visualization)
    void setReferenceMapPoints(const std::vector<MapPointPtr>& mps);
    std::vector<MapPointPtr> getReferenceMapPoints() const;

    // Clear
    void clear();

    // ID generation
    int getNextKeyframeId() { return max_kf_id_++; }
    int getNextMapPointId() { return max_mp_id_++; }

    // Statistics
    struct Statistics {
        int num_keyframes;
        int num_map_points;
        int num_observations;
        Eigen::Vector3d map_center;
        float map_scale;
    };
    Statistics getStatistics() const;

    // Save/Load
    bool save(const std::string& filename) const;
    bool load(const std::string& filename);

private:
    mutable std::mutex keyframes_mutex_;
    mutable std::mutex map_points_mutex_;

    std::map<int, KeyframePtr> keyframes_;
    std::map<int, MapPointPtr> map_points_;

    std::vector<MapPointPtr> reference_map_points_;

    int max_kf_id_;
    int max_mp_id_;
};

}  // namespace visual_slam
