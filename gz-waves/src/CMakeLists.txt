add_subdirectory(gui)
add_subdirectory(systems)

# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
# gz_get_libsources_and_unittests(sources gtest_sources)

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
  LinearRandomFFTWaveSimulation.cc
  LinearRandomFFTWaveSimulationRef.cc
  LinearRandomWaveSimulation.cc
  LinearRegularWaveSimulation.cc
  TrochoidIrregularWaveSimulation.cc
  WaveSpectrum.cc
  WaveSpreadingFunction.cc
)

set(gtest_sources
  ${gtest_sources}
  Algorithm_TEST.cc
  CGAL_TEST.cc
  EigenFFTW_TEST.cc
  Geometry_TEST.cc
  Grid_TEST.cc
  LinearRegularWaveSimulation_TEST.cc
  LinearRandomFFTWaveSimulation_TEST.cc
  MeshTools_TEST.cc
  Physics_TEST.cc
  TriangulatedGrid_TEST.cc
  Wavefield_TEST.cc
  WaveSpectrum_TEST.cc
  WaveSpreadingFunction_TEST.cc
)

# Create the library target
gz_create_core_library(SOURCES ${sources} CXX_STANDARD ${CMAKE_CXX_STANDARD})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  gz-math${GZ_MATH_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-common${GZ_COMMON_VER}::graphics
  gz-common${GZ_COMMON_VER}::profiler
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  sdformat${SDF_VER}::sdformat${SDF_VER}
  ${FFT_LIBRARIES}
)
if (${CGAL_VERSION_MAJOR} GREATER_EQUAL 6)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PUBLIC
    CGAL::CGAL
  )
endif()
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
gz_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}
    ${FFT_LIBRARIES}
)
