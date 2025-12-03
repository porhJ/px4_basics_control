cmake_minimum_required(VERSION 3.8)
project(manual_bridges)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs9 REQUIRED)

add_executable(gz_img_bridge src/gz_img_bridge.cpp)

ament_target_dependencies(gz_img_bridge
  rclcpp
  sensor_msgs
)

target_link_libraries(gz_img_bridge
  gz-transport13::core
  gz-msgs9::core
)

install(TARGETS gz_img_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(gz_caminfo_bridge src/gz_caminfo_bridge.cpp)

ament_target_dependencies(gz_caminfo_bridge
  rclcpp
  sensor_msgs)

target_link_libraries(gz_caminfo_bridge
  gz-transport13::core
  gz-msgs9::core
)

install(TARGETS
  gz_caminfo_bridge
  DESTINATION lib/${PROJECT_NAME})


ament_package()
