cmake_minimum_required(VERSION 3.8)
project(inspection_sim_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_recovery_config test/test_navigation_recovery_config.py)
endif()

install(
  DIRECTORY behavior_trees config launch maps models rviz scenes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
