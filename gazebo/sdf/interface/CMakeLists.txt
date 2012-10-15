include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources Param.cc SDF.cc parser.cc parser_deprecated.cc Converter.cc)
set (headers SDF.hh Param.hh parser.hh parser_deprecated.hh Converter.hh)

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  include_directories(SYSTEM ${XML_INCLUDE_DIRS}
                             ${urdfdom_INCLUDE_DIRS}
                             ${urdfdom_headers_INCLUDE_DIRS}
                             ${console_bridge_INCLUDE_DIRS}
                             ${CMAKE_SOURCE_DIR}/deps/opende/include)

  link_directories(${urdfdom_LIBRARY_DIRS}
                   ${urdfdom_headers_LIBRARY_DIRS}
                   ${console_bridge_LIBRARY_DIRS}
                   ${XML_LIBRARY_DIRS})

  set (sources ${sources} parser_urdf.cc)
  set (headers ${headers} parser_urdf.hh)

  # add_definitions(${XML_CFLAGS} ${urdfdom_LDFLAGS}
  # ${urdfdom_headers_LDFLAGS} ${console_bridge_LDFLAGS})

else(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  include_directories(SYSTEM ${XML_INCLUDE_DIRS})
  link_directories(${XML_LIBRARY_DIRS})
  # add_definitions(${XML_CFLAGS})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

add_definitions(${XML_CFLAGS})

gz_add_library(gazebo_sdf_interface ${sources})

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(gazebo_sdf_interface gazebo_common ${tinyxml_libraries} ${XML_LIBRARIES} ${Boost_LIBRARIES}
                                             gazebo_ode ${urdfdom_headers_LIBRARIES} ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_LDFLAGS} ${console_bridge_LDFLAGS})
else(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(gazebo_sdf_interface gazebo_common ${tinyxml_libraries} ${XML_LIBRARIES} ${Boost_LIBRARIES})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

gz_install_library(gazebo_sdf_interface)
gz_install_includes("sdf/interface" ${headers})
