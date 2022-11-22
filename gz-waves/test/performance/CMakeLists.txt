gz_get_sources(tests)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "COVERAGE")
  list(REMOVE_ITEM tests
    # add tests to remove here
  )
endif()

gz_build_tests(
  TYPE PERFORMANCE
  SOURCES ${tests}
  INCLUDE_DIRS     
    ${PROJECT_SOURCE_DIR}/src
)
