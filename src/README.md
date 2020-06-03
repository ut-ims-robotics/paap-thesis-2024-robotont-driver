# Driver architecture overview

## Communication protocol

PACKET\_ID:ARG1:ARG2:...:ARGN\r\n


## Available plugins

### plugin\_motors

This plugin subscribes to cmd\_vel (<geometry_msgs::Twist>) topic and sends the RS (Robot Speed) packet with lin\_vel\_x, lin\_vel\_y, and ang\_vel\_z arguments to the robot.


### plugin\_odom

This plugin receives ODOM packet from the robot and publishes the data on /odom (<nav_msgs::Odometry>) topic. This plugin also broadcasts odom TF to ROS.


### plugin\_power\_supply

This plugin is responsible for publishing information about battery levels, current consumption, and other hardware status indicators.


### plugin\_range

A plugin for ToF range sensor addon. Publishes Range messages.

### plugin\_led

A plugin for controlling the led strip addon. 
