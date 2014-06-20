include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources Param.cc SDF.cc parser.cc Converter.cc)
set (headers SDF.hh Param.hh parser.hh Converter.hh)
set (gtest_sources
  SDF_TEST.cc
  Converter_TEST.cc
)

gz_build_tests(${gtest_sources})

gz_add_library(gazebo_sdf_interface ${sources})

target_link_libraries(gazebo_sdf_interface
  gazebo_common ${tinyxml_libraries} ${Boost_LIBRARIES})

gz_install_library(gazebo_sdf_interface)
gz_install_includes("sdf/interface" ${headers})
