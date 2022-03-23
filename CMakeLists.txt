cmake_minimum_required(VERSION 3.5)
project(robotont_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(io_context REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/serial_driver.cpp
  src/io_context.cpp
  src/driver.cpp
  src/hardware.cpp
  src/plugin_base.cpp
  src/driver_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} "ASIO")

add_executable(driver_node src/driver.cpp src/hardware.cpp src/driver_node.cpp src/serial_port.cpp)
ament_target_dependencies(driver_node io_context serial_driver rclcpp geometry_msgs nav_msgs)

install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
