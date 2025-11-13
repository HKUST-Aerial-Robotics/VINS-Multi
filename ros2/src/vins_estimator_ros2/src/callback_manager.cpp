#include "vins_estimator_ros2/callback_manager.hpp"

#include <algorithm>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "utility/utility.h"
#include "estimator/parameters.h"

namespace vins_estimator_ros2 {

using std::placeholders::_1;
using namespace std::chrono_literals;
using vins_multi::Utility;

CallbackManager::CallbackManager(const rclcpp::Node::SharedPtr & node)
: node_(node),
  tf_broadcaster_(node) {
  const auto queue_depth = rclcpp::QoS(rclcpp::KeepLast(1000));

  // ROS2 style: publish relative topics; namespace is provided by launch
  pub_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("odometry", queue_depth);
  pub_latest_odometry_ =
    node_->create_publisher<nav_msgs::msg::Odometry>("imu_propagate", queue_depth);
  pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("path", queue_depth);
  pub_key_poses_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("key_poses", queue_depth);
  pub_keyframe_pose_ =
    node_->create_publisher<nav_msgs::msg::Odometry>("keyframe_pose", queue_depth);
  pub_margin_cloud_ =
    node_->create_publisher<sensor_msgs::msg::PointCloud>("margin_cloud", queue_depth);

  const auto module_count = vins_multi::CAM_MODULES.size();
  pub_point_cloud_.reserve(module_count);
  pub_camera_pose_.reserve(module_count);
  pub_latest_camera_pose_.reserve(module_count);
  pub_camera_pose_visual_.reserve(module_count);
  pub_extrinsic_.reserve(module_count);
  pub_image_track_.reserve(module_count);

  for (size_t i = 0; i < module_count; ++i) {
    const auto idx = std::to_string(i + 1);
    pub_point_cloud_.push_back(
      node_->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud_" + idx, queue_depth));
    pub_camera_pose_.push_back(
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose_" + idx, queue_depth));
    pub_latest_camera_pose_.push_back(
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "imu_propagate_camera_pose_" + idx, queue_depth));
    pub_camera_pose_visual_.push_back(
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "camera_pose_visual_" + idx, queue_depth));
    pub_extrinsic_.push_back(
      node_->create_publisher<nav_msgs::msg::Odometry>("extrinsic_" + idx, queue_depth));
    pub_image_track_.push_back(
      node_->create_publisher<sensor_msgs::msg::Image>("image_track_" + idx, queue_depth));
  }

  path_.header.frame_id = "world";
  cameraposevisual_.setScale(0.1);
  cameraposevisual_.setLineWidth(0.01);
}

void CallbackManager::publish_track_image_cb(const Publisher::TrackImageData & data) {
  if (data.image.empty() || data.camera_id >= pub_image_track_.size()) {
    return;
  }
  std_msgs::msg::Header header;
  header.stamp = make_stamp(data.timestamp);
  header.frame_id = "world";
  sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(header, "bgr8", data.image).toImageMsg();
  pub_image_track_[data.camera_id]->publish(*msg);
}

void CallbackManager::publish_propagate_cb(const Publisher::PropagateData & data) {
  const auto stamp = make_stamp(data.timestamp);

  // Core already outputs center-frame, low-pass filtered state. Do not filter again to
  // avoid scale shrink; just convert to IMU body pose like ROS1 visualization.cpp.
  const Eigen::Vector3d & center_pos = data.position;
  const Eigen::Vector3d & center_vel = data.velocity;
  const Eigen::Vector3d & center_omega = data.angular_velocity;
  const Eigen::Quaterniond & q_center = data.orientation;

  last_pos_ = center_pos;
  last_vel_ = center_vel;
  last_omega_ = center_omega;
  last_q_ = q_center;

  const Eigen::Vector3d & pos = center_pos;
  const Eigen::Vector3d & vel = center_vel;
  const Eigen::Vector3d & omg = center_omega;
  const Eigen::Quaterniond & q = q_center;

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "world";
  odometry.child_frame_id.clear();
  odometry.pose.pose.position.x = pos.x();
  odometry.pose.pose.position.y = pos.y();
  odometry.pose.pose.position.z = pos.z();
  odometry.pose.pose.orientation.x = q.x();
  odometry.pose.pose.orientation.y = q.y();
  odometry.pose.pose.orientation.z = q.z();
  odometry.pose.pose.orientation.w = q.w();
  odometry.twist.twist.linear.x = vel.x();
  odometry.twist.twist.linear.y = vel.y();
  odometry.twist.twist.linear.z = vel.z();
  odometry.twist.twist.angular.x = omg.x();
  odometry.twist.twist.angular.y = omg.y();
  odometry.twist.twist.angular.z = omg.z();
  pub_latest_odometry_->publish(odometry);

  // for (size_t i = 0; i < data.camera_poses.size() && i < pub_latest_camera_pose_.size(); ++i) {
  //   geometry_msgs::msg::PoseStamped cam_pose;
  //   cam_pose.header = odometry.header;
  //   cam_pose.pose.position.x = data.camera_poses[i].position[0].x();
  //   cam_pose.pose.position.y = data.camera_poses[i].position[0].y();
  //   cam_pose.pose.position.z = data.camera_poses[i].position[0].z();
  //   cam_pose.pose.orientation.x = data.camera_poses[i].orientation[0].x();
  //   cam_pose.pose.orientation.y = data.camera_poses[i].orientation[0].y();
  //   cam_pose.pose.orientation.z = data.camera_poses[i].orientation[0].z();
  //   cam_pose.pose.orientation.w = data.camera_poses[i].orientation[0].w();
  //   pub_latest_camera_pose_[i]->publish(cam_pose);

  //   cameraposevisual_.reset();
  //   cameraposevisual_.add_pose(
  //     data.camera_poses[i].position[0], data.camera_poses[i].orientation[0]);
  //   if (data.camera_poses[i].position.size() > 1) {
  //     cameraposevisual_.add_pose(
  //       data.camera_poses[i].position[1], data.camera_poses[i].orientation[1]);
  //   }
  //   cameraposevisual_.publish_by(pub_camera_pose_visual_[i], odometry.header);
  // }
  return;
}

void CallbackManager::publish_full_report_cb(const Publisher::FullReportData & data) {
  const auto stamp = make_stamp(data.timestamp);

  geometry_msgs::msg::PoseStamped cam_pose;
    cam_pose.header.stamp = make_stamp(data.camera_time);
  cam_pose.header.frame_id = "world";
  cam_pose.pose.position.x = data.camera_pose.position.x();
  cam_pose.pose.position.y = data.camera_pose.position.y();
  cam_pose.pose.position.z = data.camera_pose.position.z();
  cam_pose.pose.orientation.x = data.camera_pose.orientation.x();
  cam_pose.pose.orientation.y = data.camera_pose.orientation.y();
  cam_pose.pose.orientation.z = data.camera_pose.orientation.z();
  cam_pose.pose.orientation.w = data.camera_pose.orientation.w();
  if (data.camera_id < pub_camera_pose_.size()) {
    pub_camera_pose_[data.camera_id]->publish(cam_pose);
  }

  sensor_msgs::msg::PointCloud in_window;
  in_window.header.stamp = stamp;
  in_window.header.frame_id = "world";
  for (const auto & pt : data.in_window_pointcloud) {
    geometry_msgs::msg::Point32 p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    in_window.points.push_back(p);
  }
  if (data.camera_id < pub_point_cloud_.size()) {
    pub_point_cloud_[data.camera_id]->publish(in_window);
  }

  sensor_msgs::msg::PointCloud marginized;
  marginized.header = in_window.header;
  for (const auto & pt : data.marginized_pointcloud) {
    geometry_msgs::msg::Point32 p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    marginized.points.push_back(p);
  }
  pub_margin_cloud_->publish(marginized);

  if (data.update_latest) {
    geometry_msgs::msg::TransformStamped body_tf;
    body_tf.header.stamp = stamp;
    body_tf.header.frame_id = "world";
    body_tf.child_frame_id = "body";
    body_tf.transform.translation.x = data.TF_correct_pose.position.x();
    body_tf.transform.translation.y = data.TF_correct_pose.position.y();
    body_tf.transform.translation.z = data.TF_correct_pose.position.z();
    body_tf.transform.rotation.x = data.TF_correct_pose.orientation.x();
    body_tf.transform.rotation.y = data.TF_correct_pose.orientation.y();
    body_tf.transform.rotation.z = data.TF_correct_pose.orientation.z();
    body_tf.transform.rotation.w = data.TF_correct_pose.orientation.w();
    tf_broadcaster_.sendTransform(body_tf);

    for (size_t i = 0; i < data.pub_camera_extrinsic_params.size(); ++i) {
      geometry_msgs::msg::TransformStamped cam_tf;
      cam_tf.header.stamp = stamp;
      cam_tf.header.frame_id = "world";
      cam_tf.child_frame_id = "camera_" + std::to_string(i);
      cam_tf.transform.translation.x = data.pub_camera_extrinsic_params[i].position.x();
      cam_tf.transform.translation.y = data.pub_camera_extrinsic_params[i].position.y();
      cam_tf.transform.translation.z = data.pub_camera_extrinsic_params[i].position.z();
      cam_tf.transform.rotation.x = data.pub_camera_extrinsic_params[i].orientation.x();
      cam_tf.transform.rotation.y = data.pub_camera_extrinsic_params[i].orientation.y();
      cam_tf.transform.rotation.z = data.pub_camera_extrinsic_params[i].orientation.z();
      cam_tf.transform.rotation.w = data.pub_camera_extrinsic_params[i].orientation.w();
      tf_broadcaster_.sendTransform(cam_tf);
    }

    nav_msgs::msg::Odometry latest_odometry;
    latest_odometry.header = in_window.header;
    latest_odometry.child_frame_id = "world";
    latest_odometry.pose.pose.position.x = data.latest_frame_pose.position.x();
    latest_odometry.pose.pose.position.y = data.latest_frame_pose.position.y();
    latest_odometry.pose.pose.position.z = data.latest_frame_pose.position.z();
    latest_odometry.pose.pose.orientation.x = data.latest_frame_pose.orientation.x();
    latest_odometry.pose.pose.orientation.y = data.latest_frame_pose.orientation.y();
    latest_odometry.pose.pose.orientation.z = data.latest_frame_pose.orientation.z();
    latest_odometry.pose.pose.orientation.w = data.latest_frame_pose.orientation.w();
    latest_odometry.twist.twist.linear.x = data.latest_frame_pose.velocity.x();
    latest_odometry.twist.twist.linear.y = data.latest_frame_pose.velocity.y();
    latest_odometry.twist.twist.linear.z = data.latest_frame_pose.velocity.z();
    // Full (backend-updated) odometry must go to the odometry topic, not imu_propagate
    pub_odometry_->publish(latest_odometry);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = latest_odometry.header;
    pose_stamped.pose = latest_odometry.pose.pose;
    path_.header = latest_odometry.header;
    path_.poses.push_back(pose_stamped);
    pub_path_->publish(path_);

    visualization_msgs::msg::Marker keypose_marker;
    keypose_marker.header = latest_odometry.header;
    keypose_marker.ns = "key_poses";
    keypose_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    keypose_marker.action = visualization_msgs::msg::Marker::ADD;
    keypose_marker.scale.x = 0.05;
    keypose_marker.scale.y = 0.05;
    keypose_marker.scale.z = 0.05;
    keypose_marker.color.r = 1.0;
    keypose_marker.color.a = 1.0;
    for (const auto & key_p : data.key_poses) {
      geometry_msgs::msg::Point p;
      p.x = key_p.x();
      p.y = key_p.y();
      p.z = key_p.z();
      keypose_marker.points.push_back(p);
    }
    pub_key_poses_->publish(keypose_marker);
  }

  for (size_t i = 0; i < data.pub_camera_extrinsic_params.size() && i < pub_extrinsic_.size(); ++i) {
    nav_msgs::msg::Odometry extrinsic;
    extrinsic.header = in_window.header;
    extrinsic.pose.pose.position.x = data.pub_camera_extrinsic_params[i].position.x();
    extrinsic.pose.pose.position.y = data.pub_camera_extrinsic_params[i].position.y();
    extrinsic.pose.pose.position.z = data.pub_camera_extrinsic_params[i].position.z();
    extrinsic.pose.pose.orientation.x = data.pub_camera_extrinsic_params[i].orientation.x();
    extrinsic.pose.pose.orientation.y = data.pub_camera_extrinsic_params[i].orientation.y();
    extrinsic.pose.pose.orientation.z = data.pub_camera_extrinsic_params[i].orientation.z();
    extrinsic.pose.pose.orientation.w = data.pub_camera_extrinsic_params[i].orientation.w();
    pub_extrinsic_[i]->publish(extrinsic);
  }

  if (data.have_keyframe) {
    geometry_msgs::msg::PoseStamped keyframe_pose;
    keyframe_pose.header = in_window.header;
    keyframe_pose.pose.position.x = data.keyframe_pose.position.x();
    keyframe_pose.pose.position.y = data.keyframe_pose.position.y();
    keyframe_pose.pose.position.z = data.keyframe_pose.position.z();
    keyframe_pose.pose.orientation.x = data.keyframe_pose.orientation.x();
    keyframe_pose.pose.orientation.y = data.keyframe_pose.orientation.y();
    keyframe_pose.pose.orientation.z = data.keyframe_pose.orientation.z();
    keyframe_pose.pose.orientation.w = data.keyframe_pose.orientation.w();

    nav_msgs::msg::Odometry keyframe_odometry;
    keyframe_odometry.header = keyframe_pose.header;
    keyframe_odometry.pose.pose = keyframe_pose.pose;
    pub_keyframe_pose_->publish(keyframe_odometry);
  }
}

}  // namespace vins_estimator_ros2
