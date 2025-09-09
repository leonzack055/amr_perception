cmake_minimum_required(VERSION 3.8)
project(landmark_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# 定位到 colcon 工作空间根目录（假设当前 CMakeLists.txt 在 src/your_package 下）
get_filename_component(COLCON_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 对于同一个workspace中的cartographer，使用ament_auto_find_build_dependencies
ament_auto_find_build_dependencies()

# 关键：手动添加cartographer的包含路径
# 假设cartographer在同一个workspace的src目录下
set(CARTOGRAPHER_INCLUDE_DIRS 
  "${CMAKE_CURRENT_SOURCE_DIR}/../../cartographer/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../cartographer"
)

# Include directories
include_directories(
  include
  ${CARTOGRAPHER_INCLUDE_DIRS}
)

# 打印调试信息
message(STATUS "Cartographer include directories: ${CARTOGRAPHER_INCLUDE_DIRS}")

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

# Reflective post detector library
add_library(reflective_post_detector
  src/reflective_post_detector.cpp
)
ament_target_dependencies(reflective_post_detector
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Main node
add_executable(landmark_localization_node
  src/landmark_localization_node.cpp
)
target_include_directories(landmark_localization_node PRIVATE
  ${CARTOGRAPHER_INCLUDE_DIRS}
)
target_link_libraries(landmark_localization_node
  landmark_reader
  landmark_matcher
  reflective_post_detector
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

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()