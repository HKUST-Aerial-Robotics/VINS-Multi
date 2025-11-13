#include "vins_estimator_ros2/node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "estimator/parameters.h"
#include "utility/tic_toc.h"

namespace vins_estimator_ros2 {

namespace {
double to_double(const rclcpp::Time & time) {
  return static_cast<double>(time.nanoseconds()) * 1e-9;
}

cv::Mat convert_color(const cv::Mat & image, int code) {
  cv::Mat converted;
  cv::cvtColor(image, converted, code);
  return converted;
}
}  // namespace

VinsEstimatorNode::VinsEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vins_estimator_ros2", options) {
  // Load parameters first to initialize vins_multi::CAM_MODULES/IMU_MODULE
  load_parameters();

  // Create publishers after parameters are loaded so per-camera topics are created correctly
  auto node_shared = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
  callback_manager_ = std::make_shared<CallbackManager>(node_shared);

  // Configure estimator after parameters are ready
  estimator_.setParameter();

  estimator_.publisher_callbacks_.on_publish_propogate_odom_cb =
    [this](const Publisher::PropagateData & data) {
      callback_manager_->publish_propagate_cb(data);
    };
  estimator_.publisher_callbacks_.publish_track_image_cb =
    [this](const Publisher::TrackImageData & data) {
      callback_manager_->publish_track_image_cb(data);
    };
  // Optional: enable/disable full report publications via parameter
  estimator_.publisher_callbacks_.on_publish_full_report_cb =
    [this](const Publisher::FullReportData & data) {
      callback_manager_->publish_full_report_cb(data);
    };

  setup_subscribers();
  estimator_.start_process_thread();
  RCLCPP_INFO(this->get_logger(), "vins_estimator_ros2 initialized, waiting for data...");
}

VinsEstimatorNode::~VinsEstimatorNode() {
  estimator_.stop_process_thread();
}

void VinsEstimatorNode::load_parameters() {
  std::string default_config;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("vins_estimator_ros2");
    default_config = share_dir + std::string("/config/multi_rs_color/multi_l515_d435_color.yaml");
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to resolve default config path: %s", e.what());
  }

  const auto config_file =
    this->declare_parameter<std::string>("config_file", default_config);
  if (config_file.empty()) {
    RCLCPP_FATAL(this->get_logger(), "config_file parameter is required");
    throw std::runtime_error("config_file not set");
  }

  RCLCPP_INFO(this->get_logger(), "Loading configuration: %s", config_file.c_str());
  vins_multi::readParameters(config_file);
}

void VinsEstimatorNode::setup_subscribers() {
  // IMU uses reliable delivery per request
  auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000).reliable();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    vins_multi::IMU_MODULE.imu_topic_,
    imu_qos,
    std::bind(&VinsEstimatorNode::handle_imu, this, std::placeholders::_1));

  const auto mono_queue_depth = static_cast<size_t>(10);
  auto mono_qos = rclcpp::SensorDataQoS().keep_last(mono_queue_depth).best_effort();

  camera_subs_.clear();
  camera_subs_.reserve(vins_multi::CAM_MODULES.size());

  for (size_t i = 0; i < vins_multi::CAM_MODULES.size(); ++i) {
    auto cam = std::make_unique<CameraSubscribers>();
    cam->unique_id = static_cast<unsigned int>(i);
    cam->module_info = vins_multi::CAM_MODULES[i];

    const auto & module = cam->module_info;
    if (module.depth_ || module.stereo_) {
      rmw_qos_profile_t stereo_qos = rmw_qos_profile_sensor_data;
      stereo_qos.depth = std::max<size_t>(stereo_qos.depth, 50);
      cam->img0_sub.subscribe(this, module.img_topic_[0], stereo_qos);
      cam->img1_sub.subscribe(this, module.img_topic_[1], stereo_qos);
      cam->sync = std::make_shared<message_filters::Synchronizer<CameraSubscribers::SyncPolicy>>(
        CameraSubscribers::SyncPolicy(20), cam->img0_sub, cam->img1_sub);
      cam->sync->registerCallback(
        std::bind(
          &VinsEstimatorNode::handle_stereo_image,
          this,
          cam->unique_id,
          std::placeholders::_1,
          std::placeholders::_2));
    } else {
      cam->mono_sub = this->create_subscription<sensor_msgs::msg::Image>(
        module.img_topic_[0], mono_qos,
        [this, id = cam->unique_id](const sensor_msgs::msg::Image::SharedPtr msg) {
          handle_mono_image(id, msg);
        });
    }

    camera_subs_.push_back(std::move(cam));
  }
}

void VinsEstimatorNode::handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double t = to_double(msg->header.stamp);
  Eigen::Matrix<double, 6, 1> imu_data;
  imu_data << msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z,
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z;
  estimator_.inputIMU(t, imu_data);
}

cv_bridge::CvImagePtr VinsEstimatorNode::to_cv_image(const sensor_msgs::msg::Image::SharedPtr & msg) {
  cv_bridge::CvImagePtr ptr;
  if (msg->encoding == "8UC1") {
    auto clone = std::make_shared<sensor_msgs::msg::Image>(*msg);
    clone->encoding = sensor_msgs::image_encodings::MONO8;
    ptr = cv_bridge::toCvCopy(clone, sensor_msgs::image_encodings::MONO8);
  } else {
    ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  return ptr;
}

void VinsEstimatorNode::handle_stereo_image(
  unsigned int cam_id,
  const sensor_msgs::msg::Image::ConstSharedPtr & img0_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img1_msg) {
  if (cam_id >= camera_subs_.size()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Received stereo image for invalid camera id %u", cam_id);
    return;
  }
  auto img0 = cv_bridge::toCvCopy(img0_msg, img0_msg->encoding);
  cv_bridge::CvImagePtr img1;

  const auto & module = camera_subs_[cam_id]->module_info;
  if (module.depth_) {
    img1 = cv_bridge::toCvCopy(img1_msg, img1_msg->encoding);
    if (img1_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      img1->image.convertTo(img1->image, CV_16UC1, 0.001);
    }
  } else {
    img1 = cv_bridge::toCvCopy(img1_msg, img1_msg->encoding);
  }

  if (img0_msg->encoding == sensor_msgs::image_encodings::RGB8) {
#ifdef WITH_CUDA
    if (vins_multi::USE_GPU) {
      cv::cuda::GpuMat img_gpu, img_gray_gpu;
      img_gpu.upload(img0->image);
      cv::cuda::cvtColor(img_gpu, img_gray_gpu, cv::COLOR_RGB2GRAY);
      img_gray_gpu.download(img0->image);
    } else
#endif
    {
      cv::cvtColor(img0->image, img0->image, cv::COLOR_RGB2GRAY);
    }
  } else if (img0_msg->encoding == sensor_msgs::image_encodings::BGR8) {
#ifdef WITH_CUDA
    if (vins_multi::USE_GPU) {
      cv::cuda::GpuMat img_gpu, img_gray_gpu;
      img_gpu.upload(img0->image);
      cv::cuda::cvtColor(img_gpu, img_gray_gpu, cv::COLOR_BGR2GRAY);
      img_gray_gpu.download(img0->image);
    } else
#endif
    {
      cv::cvtColor(img0->image, img0->image, cv::COLOR_BGR2GRAY);
    }
  }

  const double stamp = to_double(img0_msg->header.stamp);
  if (module.depth_) {
    estimator_.inputImageToBuffer(cam_id, stamp, img0->image, img1->image);
  } else {
    estimator_.inputImageToBuffer(cam_id, stamp, img0->image, img1->image);
  }
}

void VinsEstimatorNode::handle_mono_image(
  unsigned int cam_id,
  const sensor_msgs::msg::Image::SharedPtr msg) {
  if (cam_id >= camera_subs_.size()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Received mono image for invalid camera id %u", cam_id);
    return;
  }
  auto img_ptr = to_cv_image(msg);
  if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(img_ptr->image, img_ptr->image, cv::COLOR_RGB2GRAY);
  } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    cv::cvtColor(img_ptr->image, img_ptr->image, cv::COLOR_BGR2GRAY);
  }
  estimator_.inputImage(cam_id, to_double(msg->header.stamp), img_ptr->image);
}

}  // namespace vins_estimator_ros2
