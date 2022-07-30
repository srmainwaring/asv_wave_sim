
set(GZ-RENDERING_LIBRARY_DIRS
  ${GZ-RENDERING_INCLUDE_DIRS}/../../../lib/gz-rendering-${GZ_RENDERING_VER}/engine-plugins)

link_directories(${GZ-RENDERING_LIBRARY_DIRS})

gz_add_system(waves-visual
  SOURCES
    WavesVisual.cc
    Ogre2DynamicMesh.cc
    Ogre2OceanGeometry.cc
    Ogre2OceanVisual.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-rendering${GZ_RENDERING_VER}-ogre2
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    GzOGRE2::GzOGRE2
    ${FFT_LIBRARIES}
)


gz_add_system(waves-model
  SOURCES
    WavesModel.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    ${FFT_LIBRARIES}
)
