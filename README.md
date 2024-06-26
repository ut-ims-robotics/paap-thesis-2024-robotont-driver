# ROS2 driver for the educational robotics platform Robotont

This repository contains the driver created for my bachelor's thesis. 
- Driver source code can be found in `robotont-driver`.
- Driver release files can be found in `robotont_driver-release`.
- Robotont release team and release repository registration can be found in `ros2-gbp-github-org` in the file `robotont.tf`.
- Driver is indexed in rosdistro. The entry can be found in `rosdistro` in the file `humble/distribution.yaml`.

## Thesis overview

At the University of Tartu's Institute of Technology, there is a mobile educational robotics platform called Robotont, which supports ROS1. The robot lacked a ROS2 driver, making it outdated in terms of functionality for working with ROS2 for both research and educational purposes.

The goal of this work was to update the Robotont to the ROS2 version, and during the update process, to improve the integration of plugins, verify the driver's architecture, and publish the driver in the package management system.

As a result of this work, a new driver was created for the ROS2 version, and its architecture was verified using examples from other robotics platform drivers. The capability to run plugins according to parameters was added, and the driver was published in package management systems using ROS index.
