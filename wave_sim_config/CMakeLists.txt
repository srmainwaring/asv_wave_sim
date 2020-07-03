cmake_minimum_required(VERSION 3.0.2)
project(wave_sim_config)

################################################################################
# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Dynamic reconfigure

generate_dynamic_reconfigure_options(
    cfg/WaveSim.cfg
)

################################################################################
# Declare catkin configuration

catkin_package()

################################################################################
# Install

catkin_install_python(PROGRAMS
    nodes/wave_sim_config_server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################################################################################
# Test
