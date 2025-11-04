cmake_minimum_required(VERSION 3.10)
project(vins_estimator_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vins_estimator CONFIG REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  message_filters
  tf2_ros
  cv_bridge
  rclcpp_components
  ament_index_cpp
)

add_library(vins_estimator_ros2_core
 src/callback_manager.cpp
 src/camera_pose_visualization.cpp
 src/node.cpp
)
target_include_directories(vins_estimator_ros2_core
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
 PRIVATE
   ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(vins_estimator_ros2_core
 vins_estimator::vins_estimator
 ${OpenCV_LIBS}
 Eigen3::Eigen
)
ament_target_dependencies(vins_estimator_ros2_core
 ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(vins_estimator_ros2_node
  src/main.cpp
)
target_link_libraries(vins_estimator_ros2_node
 vins_estimator_ros2_core
)
ament_target_dependencies(vins_estimator_ros2_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS
  vins_estimator_ros2_core
  vins_estimator_ros2_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(vins_estimator_ros2_core)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} OpenCV Eigen3 vins_estimator)

ament_package()
