cmake_minimum_required(VERSION 2.8.3)
project(asv_wave_sim_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(
  DIRECTORY launch media models world_models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
