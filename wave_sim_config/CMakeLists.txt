cmake_minimum_required(VERSION 3.0.2)
project(wave_sim_config)

################################################################################
# Compile as C++11, supported in ROS Kinetic and newer

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################################################
# Find dependent catkin and other packages

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

find_package(gazebo REQUIRED)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Dynamic reconfigure

generate_dynamic_reconfigure_options(
  cfg/Wave.cfg
  cfg/WaveSinusoid.cfg
  cfg/WaveTile.cfg
  cfg/WaveWind.cfg
)

################################################################################
# Declare catkin configuration

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    rospy
)

################################################################################
# Build

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  # ${IGNITION-COMMON_INCLUDE_DIRS}
  # ${IGNITION-MATHS_INCLUDE_DIRS}
  # ${IGNITION-MSGS_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  # ${IGNITION-COMMON_LIBRARY_DIRS}
  # ${IGNITION-MATHS_LIBRARY_DIRS}
  # ${IGNITION-MSGS_LIBRARY_DIRS}
)

# Executables
add_executable(wave_sim_config_server
    src/server.cpp
)
target_link_libraries(wave_sim_config_server PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

################################################################################
# Install

install(TARGETS
  wave_sim_config_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY
  include/wave_sim_config/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
  nodes/wave_sim_config_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)
    find_package(rosunit REQUIRED)

    # C++ tests
    catkin_add_gtest(test_server test/test_server.cpp)
    target_link_libraries(test_server ${catkin_LIBRARIES})

    # Python tests
    # catkin_add_nosetests(test/test_server.py)

    # rostest tests

endif()
