#################################################
# gz_add_system(<system>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>]
#              [PRIVATE_COMPILE_DEFS <definitions...>])
#
# Add a system component to Gazebo.
#
# <system> Required. Name of the system.
#
# SOURCES: Required. Specify the source files for the system.
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
# [PRIVATE_COMPILE_DEFS]: Specify a list of private compile definitions to add to target.
#
function(gz_add_system system_name)

  set(options)
  set(oneValueArgs)
  set(multiValueArgs SOURCES PUBLIC_LINK_LIBS PRIVATE_LINK_LIBS PRIVATE_COMPILE_DEFS)

  cmake_parse_arguments(gz_add_system "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(gz_add_system_SOURCES)
    set(sources ${gz_add_system_SOURCES})
  else()
    message(FATAL_ERROR "You must specify SOURCES for gz_add_system(~)!")
  endif()

  set(system_name ${system_name}-system)


  if(MSVC)
    # Warning #4251 is the "dll-interface" warning that tells you when types
    # used by a class are not being exported. These generated source files have
    # private members that don't get exported, so they trigger this warning.
    # However, the warning is not important since those members do not need to
    # be interfaced with.
    set_source_files_properties(${sources} COMPILE_FLAGS "/wd4251")
  endif()


  gz_add_component(${system_name}
    SOURCES ${sources}
    GET_TARGET_NAME system_target
    INDEPENDENT_FROM_PROJECT_LIB TRUE
    CXX_STANDARD 17)

  target_link_libraries(${system_target}
    PUBLIC
      ${gz_add_system_PUBLIC_LINK_LIBS}
      ${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE
      ${gz_add_system_PRIVATE_LINK_LIBS}
      gz-plugin${GZ_PLUGIN_VER}::register
  )

  if(gz_add_system_PRIVATE_COMPILE_DEFS)
    target_compile_definitions(${system_target}
      PRIVATE
        ${gz_add_system_PRIVATE_COMPILE_DEFS}
    )
  endif()

  # # Note that plugins are currently being installed in 2 places. /lib and the plugin dir
  # install(TARGETS ${system_target} DESTINATION ${GZ_SIM_PLUGIN_INSTALL_DIR})

  # # The library created by `gz_add_component` includes the gz-sim version
  # # (i.e. libgz-sim-name-system.so), but for portability of SDF
  # # files, we also install an unversioned symlink into the same versioned folder.
  # set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${system_target}${CMAKE_SHARED_LIBRARY_SUFFIX})
  # set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${system_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
  # if(WIN32)
  #   # symlinks on Windows require admin priviledges, fallback to copy
  #   ADD_CUSTOM_COMMAND(TARGET ${system_target} POST_BUILD
  #     COMMAND "${CMAKE_COMMAND}" -E copy
  #       "$<TARGET_FILE:${system_target}>"
  #       "$<TARGET_FILE_DIR:${system_target}>/${unversioned}")
  # else()
  #   file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  #   EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned} WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  #   INSTALL(FILES ${PROJECT_BINARY_DIR}/lib/${unversioned} DESTINATION ${GZ_SIM_PLUGIN_INSTALL_DIR})
  # endif()
endfunction()

add_subdirectory(dynamic)
add_subdirectory(hydrodynamics)
add_subdirectory(waves)
