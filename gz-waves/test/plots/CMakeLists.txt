
#============================================================================
# Find additional dependencies
#============================================================================

find_package(Boost COMPONENTS iostreams)

#============================================================================
# Add executables
#============================================================================

add_executable(PLOT_GnuPlotExample PLOT_GnuPlotExample.cc)
target_link_libraries(PLOT_GnuPlotExample
  ${PROJECT_LIBRARY_TARGET_NAME}
  Boost::iostreams
)
target_include_directories(PLOT_GnuPlotExample
  PUBLIC ${gnuplot-iostream_INCLUDE_DIRS}
)
target_compile_options(PLOT_GnuPlotExample
  PRIVATE "-Wno-unused-result"
)

add_executable(PLOT_LinearRandomFFTWaves PLOT_LinearRandomFFTWaves.cc)
target_link_libraries(PLOT_LinearRandomFFTWaves
  ${PROJECT_LIBRARY_TARGET_NAME}
  Boost::iostreams
)
target_include_directories(PLOT_LinearRandomFFTWaves
  PUBLIC ${gnuplot-iostream_INCLUDE_DIRS}
)
target_compile_options(PLOT_LinearRandomFFTWaves
  PRIVATE "-Wno-unused-result"
)

add_executable(PLOT_LinearRandomWaves PLOT_LinearRandomWaves.cc)
target_link_libraries(PLOT_LinearRandomWaves
  ${PROJECT_LIBRARY_TARGET_NAME}
  Boost::iostreams
)
target_include_directories(PLOT_LinearRandomWaves
  PUBLIC ${gnuplot-iostream_INCLUDE_DIRS}
)
target_compile_options(PLOT_LinearRandomWaves
  PRIVATE "-Wno-unused-result"
)

add_executable(PLOT_LinearRegularWaves PLOT_LinearRegularWaves.cc)
target_link_libraries(PLOT_LinearRegularWaves
  ${PROJECT_LIBRARY_TARGET_NAME}
  Boost::iostreams
)
target_include_directories(PLOT_LinearRegularWaves
  PUBLIC ${gnuplot-iostream_INCLUDE_DIRS}
)
target_compile_options(PLOT_LinearRegularWaves
  PRIVATE "-Wno-unused-result"
)

add_executable(PLOT_WaveSpectrum PLOT_WaveSpectrum.cc)
target_link_libraries(PLOT_WaveSpectrum
  ${PROJECT_LIBRARY_TARGET_NAME}
  Boost::iostreams
)
target_include_directories(PLOT_WaveSpectrum
  PUBLIC ${gnuplot-iostream_INCLUDE_DIRS}
)
target_compile_options(PLOT_WaveSpectrum
  PRIVATE "-Wno-unused-result"
)

#============================================================================
# Install targets
#============================================================================

install(
  TARGETS
    PLOT_GnuPlotExample
    PLOT_LinearRandomFFTWaves
    PLOT_LinearRandomWaves
    PLOT_LinearRegularWaves
    PLOT_WaveSpectrum
  RUNTIME DESTINATION bin
)
