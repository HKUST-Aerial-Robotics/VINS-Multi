cmake_minimum_required(VERSION 2.8.3)
project(camera_models)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# 添加所有源文件
add_library(camera_models SHARED
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc
)

target_link_libraries(camera_models
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

# 安装头文件和库
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(TARGETS camera_models
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)