cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(WavesControl)

set(CMAKE_AUTOMOC ON)

# Garden (default)
if("$ENV{GZ_VERSION}" STREQUAL "garden" OR NOT DEFINED "ENV{GZ_VERSION}")

  find_package(gz-sim7 REQUIRED COMPONENTS gui)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
 
  message(STATUS "Compiling against Gazebo Garden")

# Harmonic
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")

  find_package(gz-sim8 REQUIRED COMPONENTS gui)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()


QT5_ADD_RESOURCES(resources_RCC ${PROJECT_NAME}.qrc)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_NAME}.cc
  ${resources_RCC}
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE gz-sim${GZ_SIM_VER}::gui
)
