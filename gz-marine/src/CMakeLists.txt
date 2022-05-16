add_subdirectory(gui)
add_subdirectory(systems)

# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
# ign_get_libsources_and_unittests(sources gtest_sources)

# Collect source and test files manually

set(sources
  Algorithm.cc
  Convert.cc
  Geometry.cc
  Grid.cc
  MeshTools.cc
  OceanTile.cc
  PhysicalConstants.cc
  Physics.cc
  SubMeshWithTangents.cc
  TriangulatedGrid.cc
  Utilities.cc
  Wavefield.cc
  WavefieldSampler.cc
  WaveParameters.cc
  WaveSimulation.cc
  WaveSimulationFFT2.cc
  WaveSimulationFFTW.cc
  WaveSimulationOpenCL.cc
  WaveSimulationSinusoid.cc
  WaveSimulationTrochoid.cc
  WaveSpectrum.cc
)

set(gtest_sources
  ${gtest_sources}
  Algorithm_TEST.cc
  CGAL_TEST.cc
  Geometry_TEST.cc
  Grid_TEST.cc
  MeshTools_TEST.cc
  Physics_TEST.cc
  TriangulatedGrid_TEST.cc
  Wavefield_TEST.cc
  WaveSimulation_TEST.cc
  WaveSimulationFFT2_TEST.cc
)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ignition-math${IGN_MATH_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-common${IGN_COMMON_VER}::graphics
  ignition-common${IGN_COMMON_VER}::profiler
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  sdformat${SDF_VER}::sdformat${SDF_VER}
  ${TBB_LIBRARIES}
)
if (UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE stdc++fs)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(${PROJECT_SOURCE_DIR}/test)

# Build the unit tests
ign_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}
    ${FFT_LIBRARIES}
    ${TBB_LIBRARIES}
)
