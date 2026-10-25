# Graphics-dependent regression tests are opt-in, like the rendering examples.
option(GZ_WAVES_BUILD_RENDERING_TESTS "Build Ogre2 ocean lifetime tests (requires a display)" OFF)
if(GZ_WAVES_BUILD_RENDERING_TESTS)
  enable_testing()
  add_executable(ocean_lifetime_test Ogre2OceanLifetime.cc)
  target_include_directories(ocean_lifetime_test PRIVATE
    ${PROJECT_SOURCE_DIR}/src/systems/waves)
  target_link_libraries(ocean_lifetime_test PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering-ogre2
    ${PROJECT_LIBRARY_TARGET_NAME})
  foreach(kind visual geometry)
    foreach(mode shared unique)
      add_test(NAME ocean_lifetime_${kind}_${mode}
        COMMAND ocean_lifetime_test ${kind} ${mode})
      set_tests_properties(ocean_lifetime_${kind}_${mode} PROPERTIES TIMEOUT 30)
    endforeach()
  endforeach()
  add_test(NAME ocean_lifetime_scene_clone COMMAND ocean_lifetime_test scene clone)
  set_tests_properties(ocean_lifetime_scene_clone PROPERTIES TIMEOUT 30)
endif()
