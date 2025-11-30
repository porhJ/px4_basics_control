cmake_minimum_required(VERSION 3.5)
project(aruco_land)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)           
find_package(tf2_geometry_msgs REQUIRED)

# --- Vision Node ---
add_executable(aruco_vision_node src/aruco_vision_node.cpp)

ament_target_dependencies(aruco_vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  OpenCV
)

# --- Landing Node ---
add_executable(landing_node src/landing_node.cpp)

ament_target_dependencies(landing_node
  rclcpp
  geometry_msgs
  std_msgs
  px4_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  aruco_vision_node
  landing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()