find_package(HDF5 REQUIRED COMPONENTS CXX HL)
find_package(HighFive REQUIRED)

gz_add_system(linear-wave-body
  SOURCES
    LinearWaveBody.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    Eigen3::Eigen
    HighFive
    hdf5::hdf5_cpp
 )
