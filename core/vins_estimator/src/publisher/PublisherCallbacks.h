#pragma once

#include <functional>
#include <vector>
#include <map>
#include <string>
#include <list>
#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

namespace Publisher{

//------------------------------------------------------------------------------
// I. Core Reusable Data Structures
// These are the fundamental data units used across different publishing events.
//------------------------------------------------------------------------------

/**
 * @brief Represents the complete state of a rigid body (e.g., IMU, Keyframe) in the world frame.
 * @details This is the most fundamental and reusable data unit in the VIO system.
 */
struct BodyState {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d bias_g; // Gyroscope bias
    Eigen::Vector3d bias_a; // Accelerometer bias
    Eigen::Vector3d omega;  // un_gyr_

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Describes the geometric transformation (extrinsics) of a single camera relative to the IMU body.
 */
struct LatestCameraModuleExtrinsics {
    bool is_stereo;
    int module_id; // Unique ID for the camera module.
    std::vector<Eigen::Vector3d> t_imu_cam; // Translation from IMU to camera(s). 0 left 1 right
    std::vector<Eigen::Matrix3d> r_imu_cam; // Rotation from IMU to camera(s).
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Represents a 3D map point in the world frame.
 */


//------------------------------------------------------------------------------
// II. Data Packets for Specific Callbacks
// Each struct is a data packet tailored for a specific publishing event.
//------------------------------------------------------------------------------

/**
 * @brief Data packet for publishing the feature tracking debug image.
 * @details Corresponds to the `pubTrackImage` function.
 */
struct TrackImageData {
    double timestamp;
    unsigned int camera_id;
    cv::Mat image;
};

/**
 * @brief Data packet for publishing the latest odometry, typically from high-frequency IMU propagation.
 * @details Corresponds to the `pubLatestOdometry` function.
 */
struct LatestOdometryData {
    double timestamp; // Timestamp of the latest IMU measurement.
    BodyState imu_state; // The latest state of the IMU in the world frame.
    std::vector<LatestCameraModuleExtrinsics>  camara_extrinsics; // Extrinsics for all camera modules.
    // Transformation from the main IMU to the IMU cluster center (if used).
    Eigen::Vector3d center_t_imu;
    Eigen::Matrix3d center_r_imu;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief A comprehensive data packet sent after a full backend optimization.
 * @details This single packet contains all data needed for `pubOdometry`, `pubKeyPoses`, 
 *          `pubPointCloud`, `pubKeyframe`, and `pubTF`.
 */

//optimized data


// Vecotr3D R : unique_id camera pose； for  pub_camera_pose[unique_id]
// Vector 3D; unique_id caemra in win feature;

//Vector3D: marginized feature; in world frame


//all_image_frame_ptr_ latest pose
//latset camera extrinsics

// latset key poses


struct Pose{
    Pose(){}
    Pose(Eigen::Vector3d p, Eigen::Quaterniond q):position(p), orientation(q){}
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FullReportData {
    bool publish_latest;
    bool optimized_valid = false;
    bool latest_frame_is_keyframe;
    bool have_keyframe = false;
    bool update_latest; // Whether the latest frame is updated in this report.
    double timestamp; // Timestamp of the latest keyframe that triggered the report.
    uint32_t camera_id;
    double camera_time;
    Pose camera_pose;
    std::vector<Pose> pub_camera_extrinsic_params; // Poses of all cameras in world frame
    std::list<Eigen::Vector3d> in_window_pointcloud; // Features marginalized out in world frame
    std::list<Eigen::Vector3d> marginized_pointcloud; // All map points in world frame
    std::list<Eigen::Vector3d> key_poses; // Orientations of all keyframes in world frame
    Pose TF_correct_pose; // The latest optimized pose of the IMU in the world frame.
    Pose keyframe_pose; // only need translation
    BodyState latest_frame_pose;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CamerasPose{
    CamerasPose(std::vector<Eigen::Vector3d> p, std::vector<Eigen::Quaterniond> q):position(p), orientation(q){}
    std::vector<Eigen::Vector3d> position;
    std::vector<Eigen::Quaterniond> orientation;
};

struct PropagateData{
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
    std::vector<CamerasPose> camera_poses;
};


struct PublisherCallbacks
{
    /**
     * @brief Callback triggered to publish the feature tracking image.
     */
    std::function<void(const TrackImageData&)> publish_track_image_cb = nullptr;

    /**
     * @brief Callback triggered to publish the high-frequency, IMU-propagated odometry.
     */
    std::function<void(const PropagateData&)> on_publish_propogate_odom_cb = nullptr;

    /**
     * @brief Callback triggered after a backend optimization to publish the full state.
     */
    std::function<void(const FullReportData&)> on_publish_full_report_cb = nullptr;
};

} // namespace vins_multi