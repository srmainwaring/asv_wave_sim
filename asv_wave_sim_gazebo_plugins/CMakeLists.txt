cmake_minimum_required(VERSION 3.10.2)
project(ignition-marine1 VERSION 1.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 2.8.0 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math7 REQUIRED COMPONENTS eigen3)
set(IGN_MATH_VER ${ignition-math7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common5
  COMPONENTS
    graphics
    # profiler
    # events
    # av
  REQUIRED
)
set(IGN_COMMON_VER ${ignition-common5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs9 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs9_VERSION_MAJOR})

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport12 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport12_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-rendering
ign_find_package(ignition-rendering7 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gazebo
ign_find_package(ignition-gazebo7 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo7_VERSION_MAJOR})

#--------------------------------------
# Find SDFormat
ign_find_package(sdformat13 REQUIRED)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

#--------------------------------------
# Find OGRE
list(APPEND ign_ogre_components "RTShaderSystem" "Terrain" "Overlay" "Paging")

ign_find_package(IgnOGRE VERSION 1.9.0
  COMPONENTS ${ign_ogre_components}
  REQUIRED_BY ogre
  PRIVATE_FOR ogre)

  #--------------------------------------
# Find OGRE2
ign_find_package(IgnOGRE2 VERSION 2.2.0
    COMPONENTS HlmsPbs HlmsUnlit Overlay
    REQUIRED_BY ogre2
    PRIVATE_FOR ogre2)

if (OGRE2_FOUND)
  set(HAVE_OGRE2 TRUE)
endif()

#--------------------------------------
# CGAL and its components...

find_package(CGAL REQUIRED COMPONENTS Core)

if(NOT CGAL_FOUND)
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()

# include helper file
include(${CGAL_USE_FILE})

#--------------------------------------
# Other dependencies...

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

#--------------------------------------
# Find FFTW3 (double-precision) (GPL) as FFT library.

find_path(FFTW3-3_INCLUDE_DIR
  NAMES fftw3.h
  HINTS $ENV{FFTW3_DIR}/include
  PATHS /usr/local/include
        /usr/include
)

find_library(FFTW3-3_LIBRARY
  NAMES fftw3 libfftw3
  HINTS $ENV{FFTW3_DIR}/lib
  PATHS /usr/local/lib
        /usr/lib
)

add_definitions(-DUSE_FFTW3)
set(FFT_INCLUDE_DIRS ${FFTW3-3_INCLUDE_DIR})
set(FFT_LIBRARIES ${FFTW3-3_LIBRARY})

#--------------------------------------
# Find OpenCL

find_package (OpenCL REQUIRED)

#--------------------------------------
# Find clFFT

find_package (clFFT REQUIRED)

#============================================================================
# Configure the build
#============================================================================

# if (HAVE_OGRE2)
#   list(APPEND RENDERING_COMPONENTS ogre2)
# endif()

# ign_configure_build(QUIT_IF_BUILD_ERRORS
#     COMPONENTS ${RENDERING_COMPONENTS})

ign_configure_build(QUIT_IF_BUILD_ERRORS)

# add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#--------------------------------------
# include_directories(
#   ${PROJECT_SOURCE_DIR}/include
#   ${Boost_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   ${GAZEBO_INCLUDE_DIRS}
#   ${GAZEBO_MSG_INCLUDE_DIRS}
#   ${IGNITION-COMMON_INCLUDE_DIRS}
#   ${IGNITION-MATHS_INCLUDE_DIRS}
#   ${IGNITION-MSGS_INCLUDE_DIRS}
#   ${TBB_INCLUDE_DIRS}
#   ${FFT_INCLUDE_DIRS}
#   ${OpenCL_INCLUDE_DIRS}
#   ${CLFFT_INCLUDE_DIRS}
# )

# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
#   ${IGNITION-COMMON_LIBRARY_DIRS}
#   ${IGNITION-MATHS_LIBRARY_DIRS}
#   ${IGNITION-MSGS_LIBRARY_DIRS}
#   ${TBB_LIBRARY_DIRS}
#   )

#============================================================================
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
    src/OceanTile.cc
    src/PhysicalConstants.cc
    src/Physics.cc
    src/TriangulatedGrid.cc
    src/Utilities.cc
    src/Wavefield.cc
    src/WavefieldEntity.cc
    src/WavefieldSampler.cc
    src/WaveParameters.cc
    src/WaveSimulation.cc
    src/WaveSimulationFFTW.cc
    src/WaveSimulationOpenCL.cc
    src/WaveSimulationSinusoid.cc
    src/WaveSimulationTrochoid.cc
    src/WaveSpectrum.cc
)

target_include_directories(Hydrodynamics
  PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
)

# TODO  A hacky way to link against the engine-plugin
#       Need a better way to do this with support of the Ignition infrastructure...
# 
set(IGNITION-RENDERING_LIBRARY_DIRS ${IGNITION-RENDERING_INCLUDE_DIRS}/../../../lib/ign-rendering-${IGN_RENDERING_VER}/engine-plugins)
message("IGNITION-RENDERING_LIBRARY_DIRS: ${IGNITION-RENDERING_LIBRARY_DIRS}")

target_link_directories(Hydrodynamics
  PRIVATE
    ${IGNITION-RENDERING_LIBRARY_DIRS}
)

target_link_libraries(Hydrodynamics
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-common${IGN_COMMON_VER}::graphics
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
  ignition-rendering${IGN_RENDERING_VER}-ogre # the engine-plugin
  sdformat${SDF_VER}::sdformat${SDF_VER}
  ${Boost_LIBRARIES}
  ${CLFFT_LIBRARIES}
  ${FFT_LIBRARIES}
  ${OpenCL_LIBRARIES}
  ${TBB_LIBRARIES}
  IgnOGRE::IgnOGRE
)

# target_compile_options(Hydrodynamics PRIVATE "-Wno-unknown-pragmas")

# list(APPEND ASV_WAVE_GAZEBO_LIBRARIES_LIST Hydrodynamics)

#============================================================================
# Plugins...

# HydrodynamicsPlugin
# add_library(HydrodynamicsPlugin
#   SHARED
#     src/HydrodynamicsPlugin.cc
# )

# target_link_libraries(HydrodynamicsPlugin
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(HydrodynamicsPlugin PRIVATE "-Wno-unknown-pragmas -Wdeprecated-declarations")

# list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST HydrodynamicsPlugin)

# WavefieldModelPlugin
# add_library(WavefieldModelPlugin
#   SHARED
#     src/WavefieldModelPlugin.cc
# )

# target_link_libraries(WavefieldModelPlugin
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(WavefieldModelPlugin PRIVATE "-Wno-unknown-pragmas" "-Wno-deprecated-declarations")

# list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST WavefieldModelPlugin)

# WavefieldVisualPlugin
# add_library(WavefieldVisualPlugin
#   SHARED
#     src/WavefieldVisualPlugin.cc
# )

# target_link_libraries(WavefieldVisualPlugin
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(WavefieldVisualPlugin PRIVATE "-Wno-unknown-pragmas" "-Wno-deprecated-declarations")

# list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST WavefieldVisualPlugin)

# OceanVisualPlugin
# add_library(OceanVisualPlugin
#   SHARED
#     src/OceanVisual.cc
#     src/OceanVisualPlugin.cc
# )

# target_link_libraries(OceanVisualPlugin
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(OceanVisualPlugin PRIVATE "-Wno-unknown-pragmas" "-Wno-deprecated-declarations")

# list(APPEND ASV_WAVE_GAZEBO_PLUGINS_LIST OceanVisualPlugin)

#============================================================================
# Executables...

# include_directories(
#   ${GAZEBO_INCLUDE_DIRS}
# )

# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
# )

# add_executable(TestRunner
#   src/TestRunner.cc
#   src/MeshTools_TEST.cc
# )

# target_link_libraries(TestRunner  
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
#   ${CGAL_LIBRARIES}
#   ${CGAL_3RD_PARTY_LIBRARIES}
#   ${TBB_LIBRARIES}
# )

# target_compile_options(TestRunner PRIVATE "-Wno-unknown-pragmas")

# list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST TestRunner)

# add_executable(GzTestRunner
#   src/GzTestRunner.cc
#   src/GzPhysics_TEST.cc
# )

# target_link_libraries(GzTestRunner  
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
#   pthread
# )

# target_compile_options(GzTestRunner PRIVATE "-Wno-unknown-pragmas")

# list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST GzTestRunner)

# add_executable(HydrodynamicsMsgPublisher
#   src/HydrodynamicsMsgPublisher.cc
# )

# target_link_libraries(HydrodynamicsMsgPublisher  
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(HydrodynamicsMsgPublisher PRIVATE "-Wno-unknown-pragmas")

# add_dependencies(HydrodynamicsMsgPublisher
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST HydrodynamicsMsgPublisher)

# add_executable(HydrodynamicsMsgSubscriber
#   src/HydrodynamicsMsgSubscriber.cc
# )

# target_link_libraries(HydrodynamicsMsgSubscriber  
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(HydrodynamicsMsgSubscriber PRIVATE "-Wno-unknown-pragmas")

# add_dependencies(HydrodynamicsMsgSubscriber
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST HydrodynamicsMsgSubscriber)

# add_executable(WaveMsgPublisher
#   src/WaveMsgPublisher.cc
# )

# target_link_libraries(WaveMsgPublisher  
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(WaveMsgPublisher PRIVATE "-Wno-unknown-pragmas")

# add_dependencies(WaveMsgPublisher
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# add_executable(WaveMsgSubscriber
#   src/WaveMsgSubscriber.cc
# )

# target_link_libraries(WaveMsgSubscriber  
#   ${GAZEBO_LIBRARIES}
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# target_compile_options(WaveMsgSubscriber PRIVATE "-Wno-unknown-pragmas")

# add_dependencies(WaveMsgSubscriber
#   ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
# )

# list(APPEND ASV_WAVE_GAZEBO_EXECUTABLES_LIST WaveMsgSubscriber)

#============================================================================
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

# if(CATKIN_ENABLE_TESTING)

#   catkin_add_gtest(UNIT_Algorithm_TEST src/Algorithm_TEST.cc)
#   target_link_libraries(UNIT_Algorithm_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_Algorithm_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_CGAL_TEST src/CGAL_TEST.cc)
#   target_link_libraries(UNIT_CGAL_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_CGAL_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_Geometry_TEST src/Geometry_TEST.cc)
#   target_link_libraries(UNIT_Geometry_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_Geometry_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_Grid_TEST src/Grid_TEST.cc)
#   target_link_libraries(UNIT_Grid_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_Grid_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_Physics_TEST src/Physics_TEST.cc)
#   target_link_libraries(UNIT_Physics_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_Physics_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_TriangulatedGrid_TEST src/TriangulatedGrid_TEST.cc)
#   target_link_libraries(UNIT_TriangulatedGrid_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_TriangulatedGrid_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_Wavefield_TEST src/Wavefield_TEST.cc)
#   target_link_libraries(UNIT_Wavefield_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_Wavefield_TEST PRIVATE "-Wno-unknown-pragmas")

#   catkin_add_gtest(UNIT_WaveSimulation_TEST src/WaveSimulation_TEST.cc)
#   target_link_libraries(UNIT_WaveSimulation_TEST ${ASV_WAVE_GAZEBO_LIBRARIES_LIST})
#   target_compile_options(UNIT_WaveSimulation_TEST PRIVATE "-Wno-unknown-pragmas")

# endif()

#============================================================================
# Install

# install(TARGETS ${ASV_WAVE_GAZEBO_LIBRARIES_LIST}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${ASV_WAVE_GAZEBO_PLUGINS_LIST}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${ASV_WAVE_GAZEBO_EXECUTABLES_LIST}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hh"
#   PATTERN "*~" EXCLUDE
# )

# install(DIRECTORY include/
#    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN ".hh"
# )

# install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.pb.*"
#   PATTERN "*~" EXCLUDE
# )
