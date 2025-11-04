cmake_minimum_required(VERSION 3.5)
project(ensamble_description)

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Instalar urdf, launch y meshes
install(DIRECTORY meshes urdf launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

