cmake_minimum_required(VERSION 3.10)
project(my_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(my_msgs REQUIRED)

install(PROGRAMS
  scripts/command_listener.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command_listener
)

ament_package()
