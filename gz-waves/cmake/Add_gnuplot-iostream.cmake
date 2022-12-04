include(FetchContent)

set(GnuPlotIostream_BuildTests OFF CACHE INTERNAL "BuildTests OFF")
set(GnuPlotIostream_BuildExamples OFF CACHE INTERNAL "BuildExamples OFF")

FetchContent_Declare(
  gnuplot-iostream
  GIT_REPOSITORY ${gnuplot-iostream_URL}
  GIT_TAG        ${gnuplot-iostream_TAG}
)

FetchContent_MakeAvailable(gnuplot-iostream)
