include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  ConstantNoiseModel.cc
  ExponentialNoiseModel.cc
  GaussianNoiseModel.cc
  Noise.cc
  OrnsteinUhlenbeckNoiseModel.cc
  UniformNoiseModel.cc
  WienerNoiseModel.cc
)

set (headers
  ConstantNoiseModel.hh
  ExponentialNoiseModel.hh
  GaussianNoiseModel.hh
  Noise.hh
  OrnsteinUhlenbeckNoiseModel.hh
  UniformNoiseModel.hh
  WienerNoiseModel.hh
)

set (gtest_sources
  ConstantNoiseModel_TEST.cc
  ExponentialNoiseModel_TEST.cc
  GaussianNoiseModel_TEST.cc
  Noise_TEST.cc
  OrnsteinUhlenbeckNoiseModel_TEST.cc
  UniformNoiseModel_TEST.cc
  WienerNoiseModel_TEST.cc
)
gz_build_tests(${gtest_sources})

set (math_headers "" CACHE INTERNAL "noise headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(math_headers
    "Math Headers" "#include \"gazebo/sensors/noise/${hdr}\"\n")
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gznoise.hh.in
               ${CMAKE_CURRENT_BINARY_DIR}/gznoise.hh )

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_MATH")

gz_add_library(gazebo_noise ${sources})
target_link_libraries(gazebo_noise ${Boost_LIBRARIES})

gz_install_library(gazebo_noise)
gz_install_includes("noise" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/gznoise.hh)
