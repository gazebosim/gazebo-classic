include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources Param.cc SDF.cc parser.cc Converter.cc)
set (headers SDF.hh Param.hh parser.hh Converter.hh)
set (gtest_sources
  SDF_TEST.cc
)

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS}
                             ${urdfdom_headers_INCLUDE_DIRS}
                             ${console_bridge_INCLUDE_DIRS}
                             ${CMAKE_SOURCE_DIR}/deps/opende/include)

  set (sources ${sources} parser_urdf.cc)
  set (headers ${headers} parser_urdf.hh)
  # add_definitions(${urdfdom_LDFLAGS}
  # ${urdfdom_headers_LDFLAGS} ${console_bridge_LDFLAGS})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

gz_build_tests(${gtest_sources})

gz_add_library(gazebo_sdf_interface ${sources})

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(gazebo_sdf_interface
    gazebo_common
    ${tinyxml_libraries}
    ${Boost_LIBRARIES}
    gazebo_ode
    ${urdfdom_headers_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${console_bridge_LIBRARIES})
else(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(gazebo_sdf_interface
    gazebo_common ${tinyxml_libraries} ${Boost_LIBRARIES})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

gz_install_library(gazebo_sdf_interface)
gz_install_includes("sdf/interface" ${headers})
