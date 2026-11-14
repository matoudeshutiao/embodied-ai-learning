cmake_minimum_required(VERSION 3.8)
project(panda_gazebo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
