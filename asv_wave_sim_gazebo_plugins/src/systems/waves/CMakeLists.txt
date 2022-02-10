
set(IGNITION-RENDERING_LIBRARY_DIRS
  ${IGNITION-RENDERING_INCLUDE_DIRS}/../../../lib/ign-rendering-${IGN_RENDERING_VER}/engine-plugins)
# message("IGNITION-RENDERING_LIBRARY_DIRS: ${IGNITION-RENDERING_LIBRARY_DIRS}")

link_directories(${IGNITION-RENDERING_LIBRARY_DIRS})

gz_add_system(waves
  SOURCES
    Waves.cc
    OceanTile.cc
    Ogre2OceanTile.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-rendering${IGN_RENDERING_VER}-ogre2
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    IgnOGRE2::IgnOGRE2
)
