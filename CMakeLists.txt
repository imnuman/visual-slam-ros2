cmake_minimum_required(VERSION 3.8)
project(visual_slam)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)

# OpenCV
find_package(OpenCV 4 REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# GTSAM (optional, for optimization)
find_package(GTSAM QUIET)

# PCL
find_package(PCL REQUIRED COMPONENTS common io)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

if(GTSAM_FOUND)
  include_directories(${GTSAM_INCLUDE_DIR})
  add_definitions(-DUSE_GTSAM)
endif()

# Source files
set(SLAM_SOURCES
  src/slam_node.cpp
  src/tracking.cpp
  src/mapping.cpp
  src/loop_closing.cpp
  src/feature/orb_extractor.cpp
  src/feature/matcher.cpp
  src/imu/preintegration.cpp
  src/imu/initializer.cpp
  src/optimization/bundle_adjustment.cpp
  src/optimization/pose_graph.cpp
  src/frame.cpp
  src/map_point.cpp
  src/map.cpp
)

# Main SLAM library
add_library(${PROJECT_NAME}_lib SHARED
  ${SLAM_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

if(GTSAM_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib gtsam)
endif()

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  cv_bridge
  image_transport
  pcl_ros
)

# SLAM node executable
add_executable(slam_node src/slam_node.cpp)

target_link_libraries(slam_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  cv_bridge
  image_transport
  pcl_ros
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  slam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_orb_extractor test/test_orb_extractor.cpp)
  target_link_libraries(test_orb_extractor ${PROJECT_NAME}_lib)

  ament_add_gtest(test_imu_preintegration test/test_imu_preintegration.cpp)
  target_link_libraries(test_imu_preintegration ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  OpenCV
  Eigen3
)

ament_package()
