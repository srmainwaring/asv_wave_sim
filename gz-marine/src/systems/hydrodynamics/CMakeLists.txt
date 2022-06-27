
gz_add_system(hydrodynamics
  SOURCES
    Hydrodynamics.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
