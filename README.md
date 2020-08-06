# robotont\_driver
ROS driver package for robotont.

This package handles low-level communication between ROS and robotont hardware.

[![Build Status](https://travis-ci.org/robotont/robotont_driver.svg?branch=melodic-devel)](https://travis-ci.org/robotont/robotont_driver)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 1. Starting the driver
### Real robot
On a real robot, the driver is started automatically via a system service. You can perform several checks to verify that the driver is running properly.

To check the status of the service:
```bash
systemctl status clearbot.service
```

If the driver node is running you should also see '/robotont/driver\_node' in the output of:
```bash
rosnode list
```

In case any of the above checks should fail try restarting the service (see the [robotont\_support](https://github.com/robotont/robotont_support) package) or launch the driver node manually:
```bash
roslaunch robotont_driver driver_basic.launch
```

### Fake driver

In case you don't have a real robot nearby, you can run this simple dummy node that subscribes to velocity commands and publishes odometry. This driver does not know anything about physics and performs a simple integration. To start the fake driver with RViz visualization:
```bash
roslaunch robotont_driver fake_driver.launch
```
*Make sure that the fixed frame is set to "odom"!*

### Gazebo simulation

For a more physics-based experience, the real robot can be replaced with a Gazebo simulation. Please see the [robotont\_gazebo](https://github.com/robotont/robotont_gazebo) package for further information.


## 2. Moving the robot using a keyboard
The drivers for the simulated and the real robot both subscribe to a `/robotont/cmd_vel` topic, where the prefix `/robotont` specifies the namespace. To move the robot, we have publish velocity messages to this exact topic with a correct namespace prefix.

Here we use the teleop\_twist\_keyboard node, which translates command line keypresses to velocity messages and published these by default on `cmd_vel` topic. We can change the topic by using either of the following two approaches:
1. Run the node in the robotont namespace with a special argument `__ns` (preferred):
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py __ns:=robotont
```

2. Use topic remapping from the command line:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robotont/cmd_vel
```

* If teleop twist keyboard is not installed:
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```

* Follow the printout in the console to learn how to control the robot. Enjoy the ride!

### 3. Moving the robot using an Android device
* Find out your computer's IP address:
```bash
ifconfig
```
* From your Android device, go to Google Play Store and install the [ROS Control](https://play.google.com/store/apps/details?id=com.robotca.ControlApp&hl=en) app.
* Open the ROS Control app on your phone.
* Add a new robot using the plus sign in the top right corner and give it a desired name.
* Insert your computer's IP address into the Master URI field by entering the following:<br>
```bash
http://IP_address:11311
```
* Click on "Show advanced options" in the prompted window and fill in "Joystick" and "Odometry" topic names with "cmd\_vel" and "odom", respectively.
* Click OK to add the robot.
* Now you can select the robot from the list and teleoperate it using the touch joystick button.

## 4. About the driver architecture

### Communication protocol

PACKET\_ID:ARG1:ARG2:...:ARGN\r\n


### Available plugins

#### plugin\_motors

This plugin subscribes to cmd\_vel (<geometry_msgs::Twist>) topic and sends the RS (Robot Speed) packet with lin\_vel\_x, lin\_vel\_y, and ang\_vel\_z arguments to the robot.


#### plugin\_odom

This plugin receives the ODOM packet from the robot and publishes the data on /odom (<nav_msgs::Odometry>) topic. This plugin also broadcasts an odom frame via TF.


#### plugin\_power\_supply

This plugin is responsible for publishing information about battery levels, current consumption, and other hardware status indicators.


#### plugin\_range

A plugin for ToF range sensor addon. Publishes Range messages.

#### plugin\_led

A plugin for controlling the led strip addon. 
