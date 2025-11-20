cmake_minimum_required(VERSION 3.15)
project(vins_estimator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ENABLE_BACKWARD "Enable backward-cpp stacktrace support" OFF)
option(VINS_USE_CUDA "Enable CUDA support" OFF)
option(VINS_BUILD_TESTS "Build VINS estimator unit tests" OFF)

if(NOT TARGET camera_models::camera_models)
  find_package(camera_models CONFIG REQUIRED)
endif()
find_package(OpenCV REQUIRED)

find_package(spdlog REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

if(VINS_USE_CUDA)
  find_package(CUDA QUIET)
  if(CUDA_FOUND)
    message(STATUS "Building with CUDA support: ${CUDA_VERSION}")
    add_compile_definitions(WITH_CUDA)
  else()
    message(WARNING "CUDA requested but not found. Building without CUDA.")
    set(VINS_USE_CUDA OFF CACHE BOOL "" FORCE)
  endif()
endif()

if(NOT VINS_USE_CUDA)
  add_compile_definitions(CERES_NO_CUDA)
endif()

set(LIBDW "")
if(ENABLE_BACKWARD)
  add_compile_definitions(USE_BACKWARD)
  set(LIBDW "dw")
endif()

set(VINS_ESTIMATOR_SOURCES
  src/estimator/estimator.cpp
  src/estimator/feature_manager.cpp
  src/estimator/parameters.cpp
  src/featureTracker/feature_tracker.cpp
  src/factor/depthFactor.cpp
  src/factor/marginalization_factor.cpp
  src/factor/pose_manifold.cpp
  src/factor/projection_factor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp
  src/factor/projectionTwoFrameOneCamDepthFactor.cpp
  src/factor/projectionTwoFrameOneCamFactor.cpp
  src/factor/projectionTwoFrameTwoCamFactor.cpp
  src/factor/reprojectionDepthFactor.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_ex_rotation.cpp
  src/initial/initial_sfm.cpp
  src/initial/solve_5pts.cpp
  src/utility/utility.cpp
)

add_library(vins_estimator STATIC ${VINS_ESTIMATOR_SOURCES})

target_include_directories(vins_estimator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/vins_estimator>
  PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

if(TARGET spdlog::spdlog_header_only)
  set(_vins_spdlog_target spdlog::spdlog_header_only)
elseif(TARGET spdlog::spdlog)
  set(_vins_spdlog_target spdlog::spdlog)
else()
  message(FATAL_ERROR "spdlog target not found after find_package")
endif()

target_link_libraries(vins_estimator
  PUBLIC
    camera_models::camera_models
    ${OpenCV_LIBS}
    Ceres::ceres
    Eigen3::Eigen
    ${_vins_spdlog_target}
)

if(LIBDW)
  target_link_libraries(vins_estimator PUBLIC ${LIBDW})
endif()

if(VINS_USE_CUDA AND CUDA_FOUND)
  target_link_libraries(vins_estimator PUBLIC ${CUDA_LIBRARIES} ${CUDA_cusolver_LIBRARY} ${CUDA_cusparse_LIBRARY})
  target_include_directories(vins_estimator PUBLIC ${CUDA_INCLUDE_DIRS})
endif()

if(VINS_BUILD_TESTS)
  include(CTest)
  find_package(GTest REQUIRED)

  add_executable(vins_estimator_core_tests
    ../tests/estimator_lifecycle_test.cpp
  )

  target_link_libraries(vins_estimator_core_tests
    PRIVATE
      vins_estimator
      GTest::gtest_main
  )

  add_test(NAME estimator_lifecycle COMMAND vins_estimator_core_tests)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS vins_estimator
  EXPORT vins_estimatorTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vins_estimator
)

install(DIRECTORY src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vins_estimator
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(EXPORT vins_estimatorTargets
  NAMESPACE vins_estimator::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vins_estimator
  FILE vins_estimatorTargets.cmake
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/vins_estimatorConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/vins_estimatorConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/vins_estimatorConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vins_estimator
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/vins_estimatorConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/vins_estimatorConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vins_estimator
)
