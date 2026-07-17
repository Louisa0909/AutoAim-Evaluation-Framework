cmake_minimum_required(VERSION 3.16)
project(ega_offline_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EGAIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../2026_EGAIM" CACHE PATH "Path to 2026_EGAIM")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(offline_auto_aim_core STATIC
  ${EGAIM_ROOT}/tasks/auto_aim/armor.cpp
  ${EGAIM_ROOT}/tasks/auto_aim/solver.cpp
  ${EGAIM_ROOT}/tasks/auto_aim/target.cpp
  ${EGAIM_ROOT}/tasks/auto_aim/tracker.cpp
  ${EGAIM_ROOT}/tasks/auto_aim/aimer.cpp
  ${EGAIM_ROOT}/tasks/auto_aim/shooter.cpp
  ${EGAIM_ROOT}/tools/extended_kalman_filter.cpp
  ${EGAIM_ROOT}/tools/math_tools.cpp
  ${EGAIM_ROOT}/tools/trajectory.cpp
  ${EGAIM_ROOT}/tools/logger.cpp
)
target_include_directories(offline_auto_aim_core PUBLIC
  ${EGAIM_ROOT}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(offline_auto_aim_core PUBLIC
  ${OpenCV_LIBS} Eigen3::Eigen fmt::fmt spdlog::spdlog yaml-cpp
)

add_executable(auto_aim_offline apps/auto_aim_offline.cpp)
target_link_libraries(auto_aim_offline PRIVATE offline_auto_aim_core nlohmann_json::nlohmann_json)

