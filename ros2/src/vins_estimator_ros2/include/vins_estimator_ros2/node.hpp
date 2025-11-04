#pragma once

#include <memory>
#include <vector>

#include <memory>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "vins_estimator_ros2/callback_manager.hpp"

namespace vins_estimator_ros2 {

class VinsEstimatorNode : public rclcpp::Node {
public:
  explicit VinsEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VinsEstimatorNode() override;

private:
  struct CameraSubscribers {
    message_filters::Subscriber<sensor_msgs::msg::Image> img0_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img1_sub;
    using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_sub;
    unsigned int unique_id;
    vins_multi::camera_module_info module_info;
  };

  void load_parameters();
  void setup_publishers();
  void setup_subscribers();
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void handle_stereo_image(
    unsigned int cam_id,
    const sensor_msgs::msg::Image::ConstSharedPtr & img0,
    const sensor_msgs::msg::Image::ConstSharedPtr & img1);
  void handle_mono_image(
    unsigned int cam_id,
    const sensor_msgs::msg::Image::SharedPtr msg);

  cv_bridge::CvImagePtr to_cv_image(const sensor_msgs::msg::Image::SharedPtr & msg);

  vins_multi::Estimator estimator_;
  std::shared_ptr<CallbackManager> callback_manager_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<std::unique_ptr<CameraSubscribers>> camera_subs_;
};

}  // namespace vins_estimator_ros2
