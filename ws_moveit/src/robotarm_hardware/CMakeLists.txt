cmake_minimum_required(VERSION 3.8)
project(robotarm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/robotarm_hardware.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
)

# Exportar descripción del plugin
pluginlib_export_plugin_description_file(
  hardware_interface robotarm_hardware.xml
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES robotarm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

