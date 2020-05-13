cmake_minimum_required(VERSION 2.8.3)
project(robotont_driver)

# Setup Eigen3 variables
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  robotont_msgs
  tf
  cmake_modules
  serial
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    robotont_msgs
    tf
    serial
    DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver_node src/driver_node.cpp src/robotont_driver.cpp src/robotont_hardware.cpp src/plugin_base.cpp src/plugin_motors.cpp src/plugin_odom.cpp src/plugin_range.cpp src/plugin_led_module.cpp) 
add_executable(gazebo_driver_node src/gazebo_driver.cpp) 
add_executable(gazebo_odom_node src/gazebo_odom.cpp) 
add_executable (fake_odom_publisher src/fake_odom_publisher.cpp)

target_link_libraries(fake_odom_publisher ${catkin_LIBRARIES})
target_link_libraries(driver_node ${catkin_LIBRARIES})
target_link_libraries(gazebo_driver_node ${catkin_LIBRARIES})
target_link_libraries(gazebo_odom_node ${catkin_LIBRARIES})

