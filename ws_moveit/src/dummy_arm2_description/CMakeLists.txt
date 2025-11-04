cmake_minimum_required(VERSION 3.5)
project(dummy_arm2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

