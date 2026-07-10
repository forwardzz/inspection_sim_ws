cmake_minimum_required(VERSION 3.8)
project(inspection_sim_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees config launch maps models rviz scenes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
