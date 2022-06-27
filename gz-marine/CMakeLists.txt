cmake_minimum_required(VERSION 3.10.2)

#============================================================================
# Initialize the project
#============================================================================
project(gz-marine1 VERSION 1.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

#============================================================================
# Configure the project
#============================================================================
gz_configure_project()

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
gz_find_package(gz-common5
  REQUIRED COMPONENTS graphics events
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-rendering
gz_find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

#--------------------------------------
# Find gz-sim
gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

#--------------------------------------
# Find SDFormat
gz_find_package(sdformat13 REQUIRED)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

#--------------------------------------
# Find OGRE
list(APPEND gz_ogre_components "RTShaderSystem" "Terrain" "Overlay" "Paging")

gz_find_package(IgnOGRE VERSION 1.9.0
  COMPONENTS ${gz_ogre_components}
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
  gz_find_package(IgnOGRE2 VERSION 2.2.0
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

# gz_configure_build(QUIT_IF_BUILD_ERRORS
#     COMPONENTS ${RENDERING_COMPONENTS})

gz_configure_build(QUIT_IF_BUILD_ERRORS)

# add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
gz_create_packages()

#============================================================================
# Configure documentation
#============================================================================

# TODO: add documentation api.md.in files
