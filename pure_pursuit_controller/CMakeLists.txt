cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs)
find_package(tf2_ros)

# Create executable
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)

# Specify dependencies
ament_target_dependencies(pure_pursuit_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(route_publisher src/route_publisher.cpp)
ament_target_dependencies(route_publisher
  rclcpp
  geometry_msgs
  Eigen3
  visualization_msgs
  tf2_ros
  )

# Install targets
install(TARGETS
  pure_pursuit_node
  route_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  Eigen3
  visualization_msgs
  tf2_ros
)

# Add linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()