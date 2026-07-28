cmake_minimum_required(VERSION 3.8)
project(diffbot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  worlds
  config
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
