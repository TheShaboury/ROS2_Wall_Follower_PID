cmake_minimum_required(VERSION 3.5)
project(wall_follower_f1tenth)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(wall_follower_node src/wall_follower.cpp)
ament_target_dependencies(wall_follower_node rclcpp std_msgs sensor_msgs geometry_msgs)

# Install executable
install(TARGETS
  wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
