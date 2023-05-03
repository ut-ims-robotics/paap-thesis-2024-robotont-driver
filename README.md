# robotont\_driver
ROS driver package for robotont.

This package handles low-level communication between ROS and robotont hardware.

[![CI](https://github.com/robotont/robotont_driver/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/robotont/robotont_driver/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 1. Installing and starting the driver

### 1. Install Ubuntu 22.04

### 2. Install ROS 2 Rolling and create a workspace [Creating a workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#creating-a-workspace)

### 3. Pull driver repo and install ROS dependencies

Navigate to workspace root folder
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

### 4. Build the driver

```bash
colcon build
```

### 5. Source install files and start the driver

```bash
source install/local_setup.bash
ros2 run robotont_driver driver_node
```

## 2. Available plugins

### plugin\_odom

This plugin receives the ODOM packet from the robot and publishes the data on /odom (<nav_msgs::Odometry>) topic. This plugin also broadcasts an odom frame via TF.
