cmake_minimum_required(VERSION 3.10.2)

#============================================================================
# Initialize the project
#============================================================================
project(gazebo_marine1 VERSION 1.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake3 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake3_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(
  REPLACE_IGNITION_INCLUDE_PATH gz/marine
)

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
  REQUIRED COMPONENTS graphics events
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
ign_find_package(ignition-plugin2 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

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
# Find OGRE-Next
find_package(OGRE-Next QUIET)
if (OGRE-Next_FOUND)
  set(HAVE_OGRE2 TRUE)
  set(HAVE_OGRE-Next TRUE)
else()
  # Find OGRE2
  ign_find_package(IgnOGRE2 VERSION 2.2.0
      COMPONENTS HlmsPbs HlmsUnlit Overlay
      REQUIRED_BY ogre2
      PRIVATE_FOR ogre2)

  if (OGRE2_FOUND)
    set(HAVE_OGRE2 TRUE)
  endif()
endif()

#--------------------------------------
# Other dependencies...

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

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

find_package(OpenCL REQUIRED)

#--------------------------------------
# Find clFFT

find_package(clFFT REQUIRED)

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

#============================================================================
# Configure documentation
#============================================================================

# TODO: add documentation api.md.in files
