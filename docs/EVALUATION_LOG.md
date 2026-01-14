# Evaluation Log - Visual SLAM ROS2

## EuRoC MAV Benchmark (2025-01-09)

### Results

| Sequence | ATE RMSE (m) | RPE RMSE (m) | Tracking | FPS |
|----------|--------------|--------------|----------|-----|
| MH_01_easy | 0.034 | 0.021 | 98% | 42 |
| MH_02_easy | 0.041 | 0.025 | 97% | 40 |
| MH_03_medium | 0.058 | 0.032 | 94% | 38 |
| V1_01_easy | 0.029 | 0.018 | 99% | 45 |

**Average ATE RMSE: 0.041m**
**Average FPS: 41**

### Notes

- Stereo + IMU fusion enabled
- ORB features: 1000 per frame
- Real-time performance on RTX 3060

### Comparison to ORB-SLAM3

Our implementation achieves similar accuracy to ORB-SLAM3 on EuRoC sequences while maintaining ROS2 integration.
