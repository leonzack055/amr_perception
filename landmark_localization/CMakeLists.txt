cmake_minimum_required(VERSION 3.8)
project(landmark_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cartographer REQUIRED)

# Include directories
include_directories(
  include
)

add_library(landmark_reader STATIC
  src/landmark_reader.cpp
)
target_include_directories(landmark_reader PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(landmark_reader PRIVATE
  cartographer
  rclcpp::rclcpp
)

# Landmark matcher library
add_library(landmark_matcher
  src/landmark_matcher.cpp
)
ament_target_dependencies(landmark_matcher
  rclcpp
)

# Reflective post detector library (standard library version, no ROS dependencies)
add_library(reflective_post_detector
  src/reflective_post_detector.cpp
)
target_include_directories(reflective_post_detector PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# Main node
add_executable(landmark_localization_node
  src/landmark_localization_node.cpp
)

target_link_libraries(landmark_localization_node
  landmark_reader
  landmark_matcher
  reflective_post_detector
  cartographer
)
ament_target_dependencies(landmark_localization_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  cartographer_ros_msgs
  std_msgs
  visualization_msgs
)

# Install
install(TARGETS
  landmark_localization_node
  landmark_reader
  landmark_matcher
  reflective_post_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  landmark_localization_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()