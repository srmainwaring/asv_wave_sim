
set(GZ-RENDERING_LIBRARY_DIRS
  ${GZ-RENDERING_INCLUDE_DIRS}/../../../lib/gz-rendering-${GZ_RENDERING_VER}/engine-plugins)

link_directories(${GZ-RENDERING_LIBRARY_DIRS})

#################################################
gz_add_component(ogre2
  SOURCES
    Ogre2DynamicMesh.cc
    Ogre2OceanGeometry.cc
    Ogre2OceanVisual.cc
  GET_TARGET_NAME
    ogre2_target
)

target_link_directories(${ogre2_target}
  PUBLIC
    ${GZ-RENDERING_LIBRARY_DIRS}
)

target_link_libraries(${ogre2_target}
  PUBLIC
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-rendering${GZ_RENDERING_VER}-ogre2
    GzOGRE2::GzOGRE2
)

#################################################
gz_add_system(waves-visual
  SOURCES
    WavesVisual.cc
  PUBLIC_LINK_LIBS
    ${ogre2_target}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    # ${FFT_LIBRARIES}
)

#################################################
gz_add_system(waves-model
  SOURCES
    WavesModel.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    # ${FFT_LIBRARIES}
)

#################################################
gz_add_system(test-visual
  SOURCES
    TestVisual.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
