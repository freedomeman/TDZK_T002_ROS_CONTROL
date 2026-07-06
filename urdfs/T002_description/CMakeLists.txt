cmake_minimum_required(VERSION 3.8)
project(t002_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY xacro meshes worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
