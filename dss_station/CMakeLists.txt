cmake_minimum_required(VERSION 3.5)
project(dss_station)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # OpenCV not opencv

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale
)
find_package(fmt REQUIRED)

# CUDA and TensorRT
find_package(CUDA REQUIRED)
set(TENSORRT_DIR "/usr")

# Add CMake module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# ORB-SLAM3 and its dependencies
find_package(Threads REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

# -- for interface usage
find_package(dss_common REQUIRED)
# -- for interface usage

add_executable(dss_station
  src/main.cpp
  src/dss_station.cpp
  src/h264_dec.cpp
)
target_include_directories(dss_station PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_DIR}/include
)

# AFTER add_executable: Enable precompiled headers to speed up ROS2 template compilation
target_precompile_headers(dss_station PRIVATE
  <rclcpp/rclcpp.hpp>
  <std_msgs/msg/byte_multi_array.hpp>
  <geometry_msgs/msg/twist.hpp>
  <nav_msgs/msg/odometry.hpp>
)

ament_target_dependencies(dss_station
  dss_common
  rclcpp
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_msgs
  OpenCV
)

target_link_libraries(dss_station
  PkgConfig::LIBAV
  fmt::fmt
  ${ORB_SLAM3_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${TENSORRT_DIR}/lib
)

install(TARGETS dss_station
  DESTINATION lib/${PROJECT_NAME})

# ==
# Install launch files to install/share/${PROJECT_NAME}/launch/
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
# ==

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
