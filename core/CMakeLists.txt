cmake_minimum_required(VERSION 3.15)
project(vins_core LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(camera_models)
add_subdirectory(vins_estimator)
