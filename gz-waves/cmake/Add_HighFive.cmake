include(FetchContent)

FetchContent_Declare(
  HighFive
  GIT_REPOSITORY ${HighFive_URL}
  GIT_TAG        ${HighFive_TAG}
)

FetchContent_MakeAvailable(HighFive)


