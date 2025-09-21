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
find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

# Include directories
include_directories(
  include
)

add_library(landmark_reader STATIC
  src/landmark_reader.cpp
)
target_include_directories(landmark_reader PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
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

include_directories(${EIGEN3_INCLUDE_DIR})
# Landmark assigner library
add_library(landmark_assigner
  src/landmark_assigner.cpp src/common/msg_conversion.cpp
)

target_include_directories(landmark_assigner PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${EIGEN3_INCLUDE_DIR}"
  )
target_include_directories(landmark_assigner SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}"
  )
target_link_libraries(landmark_assigner PRIVATE 
  ${EIGEN3_LIBRARIES}
  absl::memory
  absl::synchronization
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${cartographer_ros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

# Reflective post detector library (standard library version, no ROS dependencies)
add_library(reflective_post_detector
  src/reflective_post_detector.cpp
)
target_include_directories(reflective_post_detector PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
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
  landmark_reader
  landmark_matcher
  landmark_assigner
  reflective_post_detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  landmark_localization_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 导出包信息（供依赖包查找）
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(landmark_reader landmark_matcher landmark_assigner reflective_post_detector)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  cartographer_ros_msgs
  std_msgs
  visualization_msgs
  cartographer
  absl
)
ament_package()