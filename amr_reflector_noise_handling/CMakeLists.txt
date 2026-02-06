cmake_minimum_required(VERSION 3.8)
project(amr_reflector_noise_handling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(absl REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters geometry sample_consensus features kdtree octree search surface)
find_package(pcl_conversions REQUIRED)

# Main detector node (using header-only implementations for new modules)
add_executable(reflector_noise_node
  src/reflector_noise_node.cpp
  src/types/msg_conversion.cpp
  src/global_reflector_tracker.cpp
  src/reflector_detector.cpp
  src/visualization_helper.cpp
)

target_include_directories(reflector_noise_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(reflector_noise_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  reflector_noise_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cartographer_ros_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_msgs
  tf2_geometry_msgs
  absl
  pcl_conversions
)

target_link_libraries(reflector_noise_node ${PCL_LIBRARIES})

# 添加rosbag Reader进行测试
add_executable(reflector_noise_bag_node
  src/reflector_noise_bag_node.cpp
  src/geometric_validator.cpp
  src/types/msg_conversion.cpp
  src/global_reflector_tracker.cpp
  src/reflector_detector.cpp
  src/visualization_helper.cpp
)

target_include_directories(reflector_noise_bag_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(reflector_noise_bag_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  reflector_noise_bag_node
  rclcpp
  rosbag2_cpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cartographer_ros_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_msgs
  tf2_geometry_msgs
  absl
  pcl_conversions
)
target_link_libraries(reflector_noise_bag_node
  ${PCL_LIBRARIES}
)



install(TARGETS reflector_noise_node reflector_noise_bag_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch config files
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()