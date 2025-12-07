# px4_basics_control

A ROS 2 workspace containing packages for basic PX4 offboard control, waypoint missions, custom Gazebo–ROS 2 bridges, and precision landing. The system is designed to ensure safe, conflict-free command flow when multiple nodes interact with a PX4 vehicle.

## Overview

### basic_control pkg
#### offboard_master
Centralized offboard control manager.  
Ensures only a single node can send commands to the drone at a time, preventing conflicts between multiple control sources.

#### mission_node
Waypoint mission executor.  
Publishes sequential position setpoints to navigate the drone along predefined mission points.

### manual_bridges pkg
Custom ROS 2 bridge package for Gazebo simulation, used when `ros_gz_bridge` cannot be utilized.  
This package contains two nodes:

- **gz_img_bridge** — Bridges Gazebo camera image topics to ROS 2 sensor_msgs/Image.  
- **gz_caminfo_bridge** — Bridges Gazebo camera info topics to ROS 2 sensor_msgs/CameraInfo.

#### aruco_land pkg
### landing_node
Precision landing controller.  
Implements target tracking and controlled descent to achieve accurate landing on markers or predefined targets.

## Requirements
- ROS 2 (Humble or later)
- PX4 Autopilot with offboard mode enabled
- Gazebo (Fortress or compatible)
- MAVROS or PX4 ROS 2 message set

## Build
```bash
colcon build --symlink-install
source install/setup.bash
