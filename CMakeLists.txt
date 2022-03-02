cmake_minimum_required(VERSION 3.5)
project(robotont_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(io_context REQUIRED)
#find_package(robotont_driver REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#add_executable(robotont_driver src/robotont_driver.cpp src/odom.cpp)
#ament_target_dependencies(robotont_driver rclcpp serial geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/serial_driver.cpp
  src/sven_driver.cpp
  src/io_context.cpp
)

ament_target_dependencies(${PROJECT_NAME} "ASIO")

add_executable(sven_driver src/sven_driver.cpp src/odom.cpp)
ament_target_dependencies(sven_driver io_context serial_driver rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)


#add_executable(publisher_member_function member_function.cpp)
#ament_target_dependencies(publisher_member_function rclcpp std_msgs)
#
#add_executable(publisher_not_composable not_composable.cpp)
#ament_target_dependencies(publisher_not_composable rclcpp std_msgs)

install(TARGETS
  sven_driver
  DESTINATION lib/${PROJECT_NAME}
)
  #publisher_member_function
  #publisher_not_composable

ament_package()
