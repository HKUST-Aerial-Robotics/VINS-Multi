cmake_minimum_required(VERSION 3.15)
project(camera_models VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set(CAMERA_MODELS_SOURCES
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
)

add_library(camera_models STATIC ${CAMERA_MODELS_SOURCES})
add_library(camera_models::camera_models ALIAS camera_models)

target_include_directories(camera_models
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(camera_models
  PUBLIC
    ${OpenCV_LIBS}
    Ceres::ceres
    Eigen3::Eigen
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS camera_models
  EXPORT camera_modelsTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT camera_modelsTargets
  NAMESPACE camera_models::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/camera_models
  FILE camera_modelsTargets.cmake
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/camera_modelsConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/camera_modelsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/camera_modelsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/camera_models
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/camera_modelsConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/camera_modelsConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/camera_models
)
