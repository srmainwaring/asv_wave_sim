
set(IGNITION-RENDERING_LIBRARY_DIRS
  ${IGNITION-RENDERING_INCLUDE_DIRS}/../../../lib/ign-rendering-${IGN_RENDERING_VER}/engine-plugins)

link_directories(${IGNITION-RENDERING_LIBRARY_DIRS})

gz_add_system(waves-visual
  SOURCES
    WavesVisual.cc
    Ogre2DynamicMesh.cc
    Ogre2OceanGeometry.cc
    Ogre2OceanVisual.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-rendering${IGN_RENDERING_VER}-ogre2
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    IgnOGRE2::IgnOGRE2
    ${FFT_LIBRARIES}
)


gz_add_system(waves-model
  SOURCES
    WavesModel.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    ${FFT_LIBRARIES}
)
