/**
 * Visual-Inertial SLAM ROS2 Node
 * Real-time visual odometry and mapping
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <deque>
#include <mutex>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace visual_slam {

/**
 * ORB Feature Extractor
 */
class ORBExtractor {
public:
    ORBExtractor(int num_features = 2000, float scale_factor = 1.2, int num_levels = 8)
        : orb_(cv::ORB::create(num_features, scale_factor, num_levels)) {}

    void detect(const cv::Mat& image,
                std::vector<cv::KeyPoint>& keypoints,
                cv::Mat& descriptors) {
        orb_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
    }

private:
    cv::Ptr<cv::ORB> orb_;
};

/**
 * Feature Matcher
 */
class FeatureMatcher {
public:
    FeatureMatcher(float ratio_threshold = 0.75f)
        : matcher_(cv::BFMatcher::create(cv::NORM_HAMMING, false)),
          ratio_threshold_(ratio_threshold) {}

    std::vector<cv::DMatch> match(const cv::Mat& desc1, const cv::Mat& desc2) {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher_->knnMatch(desc1, desc2, knn_matches, 2);

        std::vector<cv::DMatch> good_matches;
        for (const auto& m : knn_matches) {
            if (m.size() >= 2 && m[0].distance < ratio_threshold_ * m[1].distance) {
                good_matches.push_back(m[0]);
            }
        }
        return good_matches;
    }

private:
    cv::Ptr<cv::BFMatcher> matcher_;
    float ratio_threshold_;
};

/**
 * IMU Preintegration
 */
class IMUPreintegration {
public:
    struct State {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d bias_gyro;
        Eigen::Vector3d bias_accel;
    };

    IMUPreintegration(double gyro_noise = 0.004, double accel_noise = 0.04)
        : gyro_noise_(gyro_noise), accel_noise_(accel_noise) {
        reset();
    }

    void reset() {
        delta_p_.setZero();
        delta_v_.setZero();
        delta_q_ = Eigen::Quaterniond::Identity();
        dt_ = 0.0;
    }

    void integrate(const Eigen::Vector3d& gyro,
                   const Eigen::Vector3d& accel,
                   double dt,
                   const Eigen::Vector3d& bias_gyro,
                   const Eigen::Vector3d& bias_accel) {
        // Bias-corrected measurements
        Eigen::Vector3d gyro_corrected = gyro - bias_gyro;
        Eigen::Vector3d accel_corrected = accel - bias_accel;

        // Integration
        Eigen::Quaterniond dq(1.0,
                              0.5 * gyro_corrected.x() * dt,
                              0.5 * gyro_corrected.y() * dt,
                              0.5 * gyro_corrected.z() * dt);
        dq.normalize();

        delta_p_ += delta_v_ * dt + 0.5 * delta_q_ * accel_corrected * dt * dt;
        delta_v_ += delta_q_ * accel_corrected * dt;
        delta_q_ = delta_q_ * dq;
        delta_q_.normalize();

        dt_ += dt;
    }

    State predict(const State& state) const {
        State predicted;
        predicted.orientation = state.orientation * delta_q_;
        predicted.velocity = state.velocity +
                            state.orientation * delta_v_ +
                            Eigen::Vector3d(0, 0, -9.81) * dt_;
        predicted.position = state.position +
                            state.velocity * dt_ +
                            0.5 * Eigen::Vector3d(0, 0, -9.81) * dt_ * dt_ +
                            state.orientation * delta_p_;
        predicted.bias_gyro = state.bias_gyro;
        predicted.bias_accel = state.bias_accel;
        return predicted;
    }

private:
    double gyro_noise_, accel_noise_;
    Eigen::Vector3d delta_p_, delta_v_;
    Eigen::Quaterniond delta_q_;
    double dt_;
};

/**
 * Keyframe
 */
struct Keyframe {
    int id;
    double timestamp;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    Eigen::Matrix4d pose;  // T_world_camera
    std::vector<Eigen::Vector3d> map_points;
};

/**
 * Visual SLAM Node
 */
class SLAMNode : public rclcpp::Node {
public:
    SLAMNode() : Node("visual_slam"), frame_id_(0) {
        // Parameters
        declare_parameter("num_features", 2000);
        declare_parameter("use_imu", true);
        declare_parameter("keyframe_interval", 5);

        int num_features = get_parameter("num_features").as_int();
        use_imu_ = get_parameter("use_imu").as_bool();
        keyframe_interval_ = get_parameter("keyframe_interval").as_int();

        // Initialize components
        orb_extractor_ = std::make_unique<ORBExtractor>(num_features);
        matcher_ = std::make_unique<FeatureMatcher>();
        imu_preint_ = std::make_unique<IMUPreintegration>();

        // Initialize state
        current_pose_ = Eigen::Matrix4d::Identity();
        current_state_.position.setZero();
        current_state_.velocity.setZero();
        current_state_.orientation = Eigen::Quaterniond::Identity();
        current_state_.bias_gyro.setZero();
        current_state_.bias_accel.setZero();

        // Subscribers
        left_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "left/image_raw", 10,
            std::bind(&SLAMNode::leftImageCallback, this, std::placeholders::_1));

        right_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "right/image_raw", 10,
            std::bind(&SLAMNode::rightImageCallback, this, std::placeholders::_1));

        if (use_imu_) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                "imu", 100,
                std::bind(&SLAMNode::imuCallback, this, std::placeholders::_1));
        }

        // Publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("slam/odometry", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("slam/path", 10);
        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("slam/pointcloud", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "Visual SLAM node initialized");
    }

private:
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image = cv_bridge::toCvCopy(msg, "mono8")->image;

        std::lock_guard<std::mutex> lock(data_mutex_);
        left_image_ = image;
        left_timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        processFrame();
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image = cv_bridge::toCvCopy(msg, "mono8")->image;

        std::lock_guard<std::mutex> lock(data_mutex_);
        right_image_ = image;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Eigen::Vector3d gyro(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );

        Eigen::Vector3d accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );

        imu_buffer_.push_back({timestamp, gyro, accel});

        // Keep buffer bounded
        while (imu_buffer_.size() > 1000) {
            imu_buffer_.pop_front();
        }
    }

    void processFrame() {
        if (left_image_.empty()) return;

        // Extract features
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_extractor_->detect(left_image_, keypoints, descriptors);

        if (keypoints.empty()) {
            RCLCPP_WARN(get_logger(), "No features detected");
            return;
        }

        // Process IMU if available
        if (use_imu_) {
            processIMU(left_timestamp_);
        }

        // Track features
        if (prev_keypoints_.empty()) {
            // First frame
            prev_keypoints_ = keypoints;
            prev_descriptors_ = descriptors;
            frame_id_++;
            return;
        }

        // Match with previous frame
        auto matches = matcher_->match(prev_descriptors_, descriptors);

        if (matches.size() < 10) {
            RCLCPP_WARN(get_logger(), "Insufficient matches: %zu", matches.size());
            prev_keypoints_ = keypoints;
            prev_descriptors_ = descriptors;
            return;
        }

        // Extract matched points
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& m : matches) {
            pts1.push_back(prev_keypoints_[m.queryIdx].pt);
            pts2.push_back(keypoints[m.trainIdx].pt);
        }

        // Estimate motion (essential matrix)
        cv::Mat K = (cv::Mat_<double>(3, 3) <<
            fx_, 0, cx_,
            0, fy_, cy_,
            0, 0, 1);

        cv::Mat E, mask;
        E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

        cv::Mat R, t;
        cv::recoverPose(E, pts1, pts2, K, R, t, mask);

        // Update pose
        Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                delta_pose(i, j) = R.at<double>(i, j);
            }
            delta_pose(i, 3) = t.at<double>(i);
        }

        current_pose_ = current_pose_ * delta_pose;

        // Update state from pose
        current_state_.position = current_pose_.block<3, 1>(0, 3);
        current_state_.orientation = Eigen::Quaterniond(current_pose_.block<3, 3>(0, 0));

        // Publish odometry
        publishOdometry(left_timestamp_);

        // Create keyframe if needed
        if (frame_id_ % keyframe_interval_ == 0) {
            createKeyframe(keypoints, descriptors);
        }

        // Update previous frame
        prev_keypoints_ = keypoints;
        prev_descriptors_ = descriptors;
        frame_id_++;
    }

    void processIMU(double image_timestamp) {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        if (imu_buffer_.empty()) return;

        // Integrate IMU measurements up to image timestamp
        double last_timestamp = prev_timestamp_;

        for (const auto& imu : imu_buffer_) {
            if (imu.timestamp > image_timestamp) break;
            if (imu.timestamp <= last_timestamp) continue;

            double dt = imu.timestamp - last_timestamp;
            imu_preint_->integrate(
                imu.gyro, imu.accel, dt,
                current_state_.bias_gyro, current_state_.bias_accel
            );
            last_timestamp = imu.timestamp;
        }

        // Predict state
        current_state_ = imu_preint_->predict(current_state_);
        imu_preint_->reset();

        prev_timestamp_ = image_timestamp;
    }

    void createKeyframe(const std::vector<cv::KeyPoint>& keypoints,
                        const cv::Mat& descriptors) {
        Keyframe kf;
        kf.id = keyframes_.size();
        kf.timestamp = left_timestamp_;
        kf.image = left_image_.clone();
        kf.keypoints = keypoints;
        kf.descriptors = descriptors.clone();
        kf.pose = current_pose_;

        keyframes_.push_back(kf);

        RCLCPP_INFO(get_logger(), "Created keyframe %d", kf.id);
    }

    void publishOdometry(double timestamp) {
        // Odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = current_state_.position.x();
        odom_msg.pose.pose.position.y = current_state_.position.y();
        odom_msg.pose.pose.position.z = current_state_.position.z();

        odom_msg.pose.pose.orientation.w = current_state_.orientation.w();
        odom_msg.pose.pose.orientation.x = current_state_.orientation.x();
        odom_msg.pose.pose.orientation.y = current_state_.orientation.y();
        odom_msg.pose.pose.orientation.z = current_state_.orientation.z();

        odom_msg.twist.twist.linear.x = current_state_.velocity.x();
        odom_msg.twist.twist.linear.y = current_state_.velocity.y();
        odom_msg.twist.twist.linear.z = current_state_.velocity.z();

        odom_pub_->publish(odom_msg);

        // Path message
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        path_.poses.push_back(pose_stamped);
        path_.header = odom_msg.header;
        path_pub_->publish(path_);

        // TF broadcast
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom_msg.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = current_state_.position.x();
        tf.transform.translation.y = current_state_.position.y();
        tf.transform.translation.z = current_state_.position.z();
        tf.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    // Components
    std::unique_ptr<ORBExtractor> orb_extractor_;
    std::unique_ptr<FeatureMatcher> matcher_;
    std::unique_ptr<IMUPreintegration> imu_preint_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Data
    std::mutex data_mutex_, imu_mutex_;
    cv::Mat left_image_, right_image_;
    double left_timestamp_ = 0.0, prev_timestamp_ = 0.0;

    std::vector<cv::KeyPoint> prev_keypoints_;
    cv::Mat prev_descriptors_;

    struct IMUData {
        double timestamp;
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
    };
    std::deque<IMUData> imu_buffer_;

    // State
    Eigen::Matrix4d current_pose_;
    IMUPreintegration::State current_state_;
    std::vector<Keyframe> keyframes_;
    nav_msgs::msg::Path path_;

    int frame_id_;
    bool use_imu_;
    int keyframe_interval_;

    // Camera intrinsics (example values)
    double fx_ = 458.654, fy_ = 457.296;
    double cx_ = 367.215, cy_ = 248.375;
};

}  // namespace visual_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<visual_slam::SLAMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
