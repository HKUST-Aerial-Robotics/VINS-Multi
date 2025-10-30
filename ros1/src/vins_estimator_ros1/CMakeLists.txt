cmake_minimum_required(VERSION 3.10)
project(vins_estimator_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  image_transport
  nodelet
  message_filters
  sensor_msgs
)

find_package(vins_estimator CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vins_estimator_ros1_publisher vins_estimator_ros1_core vins_estimator_ros1_nodelet
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
    nodelet
    message_filters
    sensor_msgs
  DEPENDS
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(vins_estimator_ros1_publisher SHARED
  src/visualization.cpp
  src/CameraPoseVisualization.cpp
  src/CallbackManager.cpp
)
target_link_libraries(vins_estimator_ros1_publisher
  PUBLIC
    vins_estimator::vins_estimator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(vins_estimator_ros1_core SHARED
  src/rosNode.cpp
)
target_link_libraries(vins_estimator_ros1_core
  PUBLIC
    vins_estimator::vins_estimator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  PRIVATE
    vins_estimator_ros1_publisher
)

add_executable(vins_estimator_ros1_node
  src/rosNodeMain.cpp
)
target_link_libraries(vins_estimator_ros1_node
  vins_estimator_ros1_core
  vins_estimator_ros1_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_library(vins_estimator_ros1_nodelet SHARED
  src/rosNodelet.cpp
)
target_link_libraries(vins_estimator_ros1_nodelet
  vins_estimator_ros1_core
  vins_estimator_ros1_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  vins_estimator_ros1_publisher
  vins_estimator_ros1_core
  vins_estimator_ros1_nodelet
  vins_estimator_ros1_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
