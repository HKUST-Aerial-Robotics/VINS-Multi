#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "vins_estimator_ros2/node.hpp"

int main(int argc, char ** argv) {
  printf("Starting vins_estimator_ros2 node...\n");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vins_estimator_ros2::VinsEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
