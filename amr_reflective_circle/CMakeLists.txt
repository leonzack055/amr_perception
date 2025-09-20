cmake_minimum_required(VERSION 3.8)
project(amr_reflective_circle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(absl REQUIRED)
find_package(tf2_eigen REQUIRED)
# 引用landmark_localization
find_package(landmark_localization REQUIRED)

## 点聚类法
# add_executable(reflector_detector_circle src/reflector_detector_circle.cpp src/match_assigner.cpp)
# target_include_directories(reflector_detector_circle PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
#   )
# target_compile_features(reflector_detector_circle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   reflector_detector_circle
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   cartographer_ros_msgs
#   visualization_msgs
#   tf2
#   tf2_ros
#   absl
#   landmark_localization
# )

# 近似拟合圆优化方法
add_executable(reflector_detector_circle_fit src/reflector_detector_circle_fit.cpp)
target_include_directories(reflector_detector_circle_fit PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(reflector_detector_circle_fit
  landmark_localization::reflective_post_detector
  landmark_localization::landmark_assigner
)
target_compile_features(reflector_detector_circle_fit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  reflector_detector_circle_fit
  rclcpp
  sensor_msgs
  geometry_msgs
  cartographer_ros_msgs
  visualization_msgs
  tf2
  tf2_ros
  absl
  landmark_localization
)

install(TARGETS reflector_detector_circle_fit
  DESTINATION lib/${PROJECT_NAME})
  
# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
