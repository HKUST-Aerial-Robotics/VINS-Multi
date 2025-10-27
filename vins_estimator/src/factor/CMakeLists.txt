cmake_minimum_required(VERSION 2.8.3)
project(factor_lib)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ENABLE_BACKWARD false)
set(CUDA false)

find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres 2.2.0 REQUIRED)

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

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
include_directories(
  ${PROJECT_SOURCE_DIR}/../
)

add_library(factor_lib_multi SHARED
    # pose_local_parameterization.cpp
    pose_manifold.cpp
    projectionTwoFrameOneCamFactor.cpp
    depthFactor.cpp
    projectionTwoFrameOneCamDepthFactor.cpp
    marginalization_factor.cpp
    projectionTwoFrameTwoCamFactor.cpp
    projectionOneFrameTwoCamFactor.cpp
)
target_link_libraries(factor_lib_multi
    ${OpenCV_LIBS} ${CERES_LIBRARIES} ${LIBDW})