#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include "publisher/PublisherCallbacks.h"
#include "estimator/parameters.h"
#include "vins_estimator_ros2/camera_pose_visualization.hpp"

namespace vins_estimator_ros2 {

class CallbackManager {
public:
  explicit CallbackManager(const rclcpp::Node::SharedPtr & node);

  void publish_propagate_cb(const Publisher::PropagateData & data);
  void publish_track_image_cb(const Publisher::TrackImageData & data);
  void publish_full_report_cb(const Publisher::FullReportData & data);

private:
  // Construct a builtin ROS time stamp from seconds using the node's clock type
  inline rclcpp::Time make_stamp(double seconds) const {
    // Preserve ROS time when /use_sim_time is enabled to avoid offset/lag
    const auto clock_type = node_->get_clock()->get_clock_type();
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), clock_type);
  }

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_latest_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr> pub_point_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_margin_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_key_poses_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keyframe_pose_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_camera_pose_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_latest_camera_pose_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> pub_camera_pose_visual_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pub_extrinsic_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_image_track_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  nav_msgs::msg::Path path_;
  vins_multi::CameraPoseVisualization cameraposevisual_{1.0f, 0.0f, 0.0f, 1.0f};

  double sum_of_path_ = 0.0;
  Eigen::Vector3d last_path_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_omega_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();

  const double interpolation_alpha_ = 0.5;
};

}  // namespace vins_estimator_ros2
