# robotont_driver
ROS driver package for robotont.

This package handles low-level communication between ROS and robotont hardware.

Visualizing the robot's movement on Rviz:
- Open terminal window: roslaunch robotont_driver fake_driver.launch
- Make sure that fixed frame is set to "odom"!
- Open another terminal window: rosrun teleop_twist_keyboard teleop_twist_keyboard.py
- If teleop twist keyboard is not installed: sudo apt-get install ros-melodic-teleop-twist-keyboard
