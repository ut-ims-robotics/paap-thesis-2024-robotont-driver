# robotont_driver
ROS driver package for robotont.

This package handles low-level communication between ROS and robotont hardware.

## Visualizing the robot's movement on Rviz:

### 1. Run fake_driver.launch
* ```roslaunch robotont_driver fake_driver.launch```
* Make sure that fixed frame is set to "odom"!

### 2. Moving the robot using teleop twist keyboard
* Open a new terminal window: <br/>
```rosrun teleop_twist_keyboard teleop_twist_keyboard.py```
* If teleop twist keyboard is not installed:<br/>
```sudo apt-get install ros-melodic-teleop-twist-keyboard```

### 3. Moving the robot using an Android device
* Find out your computer's ip-address:<br/>
```ifconfig```
* From your Android device, go to Google Play Store and install the [ROS Control](https://play.google.com/store/apps/details?id=com.robotca.ControlApp&hl=en) app.
* Open the ROS Control app on your phone.
* Add a new robot using the plus sign in the top right corner and give it a desired name.
* Insert your computer's IP address into Master URI field by entering the following:<br/>
``` http://IP_address:11311 ```
* Click on "Show advanced options" in the prompted window and fill in "Joystick" and "Odometry" topic names with "cmd_vel" and "odom", respectively.
* Click OK to add the robot.
* Now you can select the robot from the list and teleoperate it using the touch joystick button.
