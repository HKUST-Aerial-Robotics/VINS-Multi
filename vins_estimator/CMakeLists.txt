cmake_minimum_required(VERSION 2.8.3)
project(vins_multi)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ENABLE_BACKWARD false)
set(CUDA false)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    # camera_models
    image_transport
    nodelet
    message_filters
    sensor_msgs)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

if (CUDA)
  find_package(CUDA QUIET)
  if (CUDA_FOUND)
    message("-- Found CUDA version ${CUDA_VERSION}: "
        "${CUDA_LIBRARIES};"
        "${CUDA_cusolver_LIBRARY};"
        "${CUDA_cusparse_LIBRARY}")
    include_directories(
      ${CUDA_INCLUDE_DIRS}
    )
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      message("embed_platform on")
      include_directories(/usr/local/cuda/targets/aarch64-linux/include)
      link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
    else()
      message("embed_platform off")
      link_directories(/usr/local/cuda-11.4/targets/x86_64-linux/lib/)
    endif()

  else (CUDA_FOUND)
    message("-- Did not find CUDA library, disabling CUDA support.")
    update_cache_variable(CUDA OFF)
    list(APPEND CERES_COMPILE_OPTIONS CERES_NO_CUDA)
  endif (CUDA_FOUND)
else (CUDA)
  message("-- Building without CUDA.")
  list(APPEND CERES_COMPILE_OPTIONS CERES_NO_CUDA)
endif (CUDA)

if(CUDA)
    add_compile_definitions(WITH_CUDA)
endif (CUDA)

if(ENABLE_BACKWARD)
    add_definitions(-D USE_BACKWARD)
    set(LIBDW "dw")
else()
    set(LIBDW "")
endif()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}/../camera_models/include
)

add_library(parameter_lib_multi SHARED
    src/estimator/parameters.cpp
)
target_link_libraries(parameter_lib_multi
    ${OpenCV_LIBS} ${LIBDW})

add_library(utility_lib_multi
  src/utility/utility.cpp
)
target_link_libraries(utility_lib_multi
  ${LIBDW})


add_library(frontend_lib_multi SHARED
    src/featureTracker/feature_tracker.cpp
)
target_link_libraries(frontend_lib_multi
    parameter_lib_multi ${OpenCV_LIBS} ${LIBDW})

add_library(init_lib_multi SHARED
    src/initial/initial_aligment.cpp
)
target_link_libraries(init_lib_multi
    ${LIBDW})

add_library(factor_lib_multi SHARED
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/depthFactor.cpp
    src/factor/projectionTwoFrameOneCamDepthFactor.cpp
    src/factor/marginalization_factor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
)
target_link_libraries(factor_lib_multi
    camera_models ${OpenCV_LIBS} ${CERES_LIBRARIES} ${LIBDW} ${spdlog_LIBRARIES})

add_library(estimator_lib_multi SHARED
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
)

target_link_libraries(estimator_lib_multi
    parameter_lib_multi utility_lib_multi init_lib_multi factor_lib_multi ${OpenCV_LIBS} ${CERES_LIBRARIES} ${LIBDW})


#ROS related
add_library(ros1_publisher
    src/publisher/ros1/visualization.cpp
    src/publisher/ros1/CameraPoseVisualization.cpp
)
target_link_libraries(ros1_publisher
    ${catkin_LIBRARIES} ${LIBDW})
  
add_library(rosnode_lib_multi SHARED
    src/ros_node/ros1/rosNode.cpp
)
target_link_libraries(rosnode_lib_multi ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBDW})

add_executable(${PROJECT_NAME} src/ros_node/ros1/rosNodeMain.cpp)
target_link_libraries(${PROJECT_NAME} rosnode_lib_multi parameter_lib_multi utility_lib_multi estimator_lib_multi frontend_lib_multi init_lib_multi factor_lib_multi ros1_publisher ${LIBDW})

add_library(${PROJECT_NAME}_nodelet_lib src/ros_node/ros1/rosNodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet_lib rosnode_lib_multi parameter_lib_multi utility_lib_multi estimator_lib_multi frontend_lib_multi init_lib_multi factor_lib_multi ${LIBDW})