cmake_minimum_required(VERSION 2.8.3)
project(asv_wave_sim_gazebo_plugins)

###############################################################################
# Compile as C++11, supported in ROS Kinetic and newer

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############################################################################
# CGAL and its components...

find_package(CGAL REQUIRED COMPONENTS Core)

if(NOT CGAL_FOUND)
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()

# include helper file
include(${CGAL_USE_FILE})
# message("CGAL_USE_FILE: " ${CGAL_USE_FILE})

###############################################################################
# Other dependencies...

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

############################################################################### 
# Catkin...

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    Hydrodynamics
    HydrodynamicsPlugin
    WavefieldModelPlugin
    WavefieldVisualPlugin
  DEPENDS
    gazebo_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  ${TBB_LIBRARY_DIRS}
  )

############################################################################### 
# Libraries...

# Hydrodynamics
add_library(Hydrodynamics
  SHARED
    src/Algorithm.cc
    src/Convert.cc
    src/Gazebo.cc
    src/Geometry.cc
    src/Grid.cc
    src/MeshTools.cc
    src/PhysicalConstants.cc
    src/Physics.cc
    src/Utilities.cc
    src/Wavefield.cc
    src/WavefieldEntity.cc
)

target_link_libraries(Hydrodynamics
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${TBB_LIBRARIES}
)

target_compile_options(Hydrodynamics PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_WAVE_GAZEBO_LIBRARIES_LIST Hydrodynamics)

############################################################################### 
# Plugins...

# HydrodynamicsPlugin
add_library(HydrodynamicsPlugin
  SHARED
    src/HydrodynamicsPlugin.cc
)

target_link_libraries(HydrodynamicsPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(HydrodynamicsPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST HydrodynamicsPlugin)

# WavefieldModelPlugin
add_library(WavefieldModelPlugin
  SHARED
    src/WavefieldModelPlugin.cc
)

target_link_libraries(WavefieldModelPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(WavefieldModelPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST WavefieldModelPlugin)

# WavefieldVisualPlugin
add_library(WavefieldVisualPlugin
  SHARED
    src/WavefieldVisualPlugin.cc
)

target_link_libraries(WavefieldVisualPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(WavefieldVisualPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST WavefieldVisualPlugin)

############################################################################### 
# Executables...

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(TestRunner
  src/TestRunner.cc
  src/CGAL_TEST.cc
  src/MeshTools_TEST.cc
)

target_link_libraries(TestRunner  
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
  ${CGAL_LIBRARIES}
  ${CGAL_3RD_PARTY_LIBRARIES}
  ${TBB_LIBRARIES}
)

target_compile_options(TestRunner PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST TestRunner)

add_executable(GzTestRunner
  src/GzTestRunner.cc
  src/GzPhysics_TEST.cc
)

target_link_libraries(GzTestRunner  
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
  pthread
)

target_compile_options(GzTestRunner PRIVATE "-Wno-unknown-pragmas")

list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST GzTestRunner)

add_executable(HydrodynamicsMsgPublisher
  src/HydrodynamicsMsgPublisher.cc
)

target_link_libraries(HydrodynamicsMsgPublisher  
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(HydrodynamicsMsgPublisher PRIVATE "-Wno-unknown-pragmas")

add_dependencies(HydrodynamicsMsgPublisher
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST HydrodynamicsMsgPublisher)

add_executable(HydrodynamicsMsgSubscriber
  src/HydrodynamicsMsgSubscriber.cc
)

target_link_libraries(HydrodynamicsMsgSubscriber  
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(HydrodynamicsMsgSubscriber PRIVATE "-Wno-unknown-pragmas")

add_dependencies(HydrodynamicsMsgSubscriber
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST HydrodynamicsMsgSubscriber)

add_executable(WaveMsgPublisher
  src/WaveMsgPublisher.cc
)

target_link_libraries(WaveMsgPublisher  
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(WaveMsgPublisher PRIVATE "-Wno-unknown-pragmas")

add_dependencies(WaveMsgPublisher
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

add_executable(WaveMsgSubscriber
  src/WaveMsgSubscriber.cc
)

target_link_libraries(WaveMsgSubscriber  
  ${GAZEBO_LIBRARIES}
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(WaveMsgSubscriber PRIVATE "-Wno-unknown-pragmas")

add_dependencies(WaveMsgSubscriber
  ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
)

list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST WaveMsgSubscriber)

############################################################################### 
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(UNIT_Algorithm_TEST src/Algorithm_TEST.cc)
  target_link_libraries(UNIT_Algorithm_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
  target_compile_options(UNIT_Algorithm_TEST PRIVATE "-Wno-unknown-pragmas")

  catkin_add_gtest(UNIT_Geometry_TEST src/Geometry_TEST.cc)
  target_link_libraries(UNIT_Geometry_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
  target_compile_options(UNIT_Geometry_TEST PRIVATE "-Wno-unknown-pragmas")

  catkin_add_gtest(UNIT_Grid_TEST src/Grid_TEST.cc)
  target_link_libraries(UNIT_Grid_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
  target_compile_options(UNIT_Grid_TEST PRIVATE "-Wno-unknown-pragmas")

  catkin_add_gtest(UNIT_Physics_TEST src/Physics_TEST.cc)
  target_link_libraries(UNIT_Physics_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
  target_compile_options(UNIT_Physics_TEST PRIVATE "-Wno-unknown-pragmas")

  catkin_add_gtest(UNIT_Wavefield_TEST src/Wavefield_TEST.cc)
  target_link_libraries(UNIT_Wavefield_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
  target_compile_options(UNIT_Wavefield_TEST PRIVATE "-Wno-unknown-pragmas")

endif()

############################################################################### 
# Install

install(TARGETS ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${ASV_WAVE_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${ASV_WAVE_GAZEBO_EXECUTABLES_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)
