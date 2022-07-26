
set(GZ-RENDERING_LIBRARY_DIRS
  ${GZ-RENDERING_INCLUDE_DIRS}/../../../lib/gz-rendering-${GZ_RENDERING_VER}/engine-plugins)

link_directories(${GZ-RENDERING_LIBRARY_DIRS})

gz_add_system(dynamic-geometry
  SOURCES
    DynamicGeometry.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-rendering${GZ_RENDERING_VER}-ogre2
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    IgnOGRE2::IgnOGRE2
)
