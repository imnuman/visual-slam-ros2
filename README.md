# Visual SLAM ROS2

Visual-Inertial SLAM system for autonomous robot navigation with loop closure and relocalization.

## Overview

A real-time Visual-Inertial Odometry (VIO) and SLAM system built on ROS2 Humble. Combines ORB features with IMU preintegration for robust state estimation in GPS-denied environments.

## Architecture

```
┌────────────────────────────────────────────────────────────────────────────┐
│                          Visual-Inertial SLAM                               │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Sensors                  Frontend                    Backend              │
│  ┌────────┐           ┌──────────────┐           ┌──────────────┐         │
│  │Stereo  │──────────▶│   Feature    │           │   Factor     │         │
│  │Camera  │           │  Extraction  │──────────▶│   Graph      │         │
│  └────────┘           │   (ORB)      │           │   (GTSAM)    │         │
│                       └──────────────┘           └──────────────┘         │
│  ┌────────┐           ┌──────────────┐                  │                  │
│  │  IMU   │──────────▶│    IMU       │                  │                  │
│  │        │           │Preintegration│──────────────────┘                  │
│  └────────┘           └──────────────┘                                     │
│                                                                             │
│                       ┌──────────────┐           ┌──────────────┐         │
│                       │    Loop      │◀─────────▶│   Map        │         │
│                       │   Closure    │           │  Management  │         │
│                       │   (DBoW2)    │           │              │         │
│                       └──────────────┘           └──────────────┘         │
│                              │                          │                  │
│                              ▼                          ▼                  │
│                       ┌──────────────┐           ┌──────────────┐         │
│                       │ Relocalization│          │   3D Map     │         │
│                       │              │           │ Point Cloud  │         │
│                       └──────────────┘           └──────────────┘         │
│                                                                             │
└────────────────────────────────────────────────────────────────────────────┘
```

## Features

- **Visual-Inertial Odometry**: Tightly-coupled IMU + camera fusion
- **Stereo/RGBD Support**: Works with stereo cameras and depth sensors
- **Loop Closure**: DBoW2-based place recognition
- **Relocalization**: Recovery from tracking failures
- **Multi-Map**: Support for multiple map sessions
- **Dense Mapping**: Optional dense reconstruction
- **ROS2 Native**: Built for ROS2 Humble/Iron

## Supported Sensors

| Type | Models |
|------|--------|
| Stereo | Intel RealSense D435i, ZED 2, OAK-D |
| Mono+IMU | Basler + BMI088, ArduCam + MPU6050 |
| RGBD | Azure Kinect, RealSense D455 |

## Installation

```bash
# Install dependencies
sudo apt install ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-tf2-ros ros-humble-pcl-ros libgtsam-dev

# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/imnuman/visual-slam-ros2.git
git clone https://github.com/imnuman/visual-slam-ros2-msgs.git

# Build
cd ~/ros2_ws
colcon build --packages-select visual_slam visual_slam_msgs

# Source workspace
source install/setup.bash
```

## Quick Start

### With RealSense D435i

```bash
# Launch camera driver
ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=true enable_accel:=true \
    unite_imu_method:=2 enable_infra1:=true enable_infra2:=true

# Launch SLAM
ros2 launch visual_slam slam.launch.py camera:=realsense mode:=stereo_imu
```

### With ZED Camera

```bash
# Launch ZED driver
ros2 launch zed_wrapper zed2.launch.py

# Launch SLAM
ros2 launch visual_slam slam.launch.py camera:=zed mode:=stereo_imu
```

### With Custom Camera

```bash
# Launch SLAM with custom topics
ros2 launch visual_slam slam.launch.py \
    left_image:=/camera/left/image_raw \
    right_image:=/camera/right/image_raw \
    imu:=/imu/data \
    mode:=stereo_imu
```

## Configuration

```yaml
# config/slam_config.yaml
slam:
  # Feature detection
  feature:
    type: "ORB"
    num_features: 2000
    scale_factor: 1.2
    num_levels: 8

  # IMU parameters
  imu:
    gyro_noise: 0.004
    accel_noise: 0.04
    gyro_bias_noise: 0.00004
    accel_bias_noise: 0.004

  # Optimization
  optimization:
    window_size: 10
    keyframe_interval: 5
    use_imu: true

  # Loop closure
  loop_closure:
    enabled: true
    min_score: 0.05
    min_inliers: 25

  # Mapping
  mapping:
    create_dense_map: false
    voxel_size: 0.05
    map_update_rate: 1.0
```

## ROS2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/slam/odometry` | nav_msgs/Odometry | Camera pose |
| `/slam/path` | nav_msgs/Path | Trajectory |
| `/slam/pointcloud` | sensor_msgs/PointCloud2 | Sparse map |
| `/slam/keyframe` | visual_slam_msgs/KeyFrame | Keyframe data |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/slam/save_map` | std_srvs/Trigger | Save map to file |
| `/slam/load_map` | visual_slam_msgs/LoadMap | Load existing map |
| `/slam/reset` | std_srvs/Trigger | Reset SLAM |

## Project Structure

```
visual-slam-ros2/
├── src/
│   ├── slam_node.cpp           # Main ROS2 node
│   ├── tracking.cpp            # Visual tracking
│   ├── mapping.cpp             # Local mapping
│   ├── loop_closing.cpp        # Loop closure detection
│   ├── feature/
│   │   ├── orb_extractor.cpp   # ORB feature extraction
│   │   └── matcher.cpp         # Feature matching
│   ├── imu/
│   │   ├── preintegration.cpp  # IMU preintegration
│   │   └── initializer.cpp     # VIO initialization
│   └── optimization/
│       ├── bundle_adjustment.cpp
│       └── pose_graph.cpp
├── include/
│   └── visual_slam/
│       └── *.hpp
├── config/
│   ├── slam_config.yaml
│   └── camera/
│       ├── realsense_d435i.yaml
│       └── zed2.yaml
├── launch/
│   └── slam.launch.py
├── rviz/
│   └── slam.rviz
└── CMakeLists.txt
```

## Performance

| Metric | Value |
|--------|-------|
| Tracking FPS | 30 Hz (stereo) |
| ATE (EuRoC MH01) | 0.035 m |
| CPU Usage | ~150% (4 threads) |
| Memory | ~500 MB |

## Evaluation

```bash
# Run on EuRoC dataset
ros2 launch visual_slam euroc.launch.py sequence:=MH_01_easy

# Evaluate trajectory
evo_ape euroc MH_01_easy.csv slam_trajectory.csv -a --plot
```

## References

- [ORB-SLAM3](https://arxiv.org/abs/2007.11898)
- [VINS-Fusion](https://arxiv.org/abs/1901.01657)
- [GTSAM](https://gtsam.org/)
- [EuRoC Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

## License

GPL-3.0 License - see [LICENSE](LICENSE) for details.
