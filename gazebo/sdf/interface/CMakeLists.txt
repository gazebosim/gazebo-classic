include_directories(SYSTEM ${XML_INCLUDE_DIRS})
link_directories(${XML_LIBRARY_DIRS})
add_definitions(${XML_CFLAGS})

set (sources Param.cc SDF.cc parser.cc parser_deprecated.cc Converter.cc)
set (headers SDF.hh Param.hh parser.hh parser_deprecated.hh Converter.hh)

gz_add_library(gazebo_sdf_interface ${sources})
target_link_libraries(gazebo_sdf_interface gazebo_common ${tinyxml_libraries} ${XML_LIBRARIES} ${Boost_LIBRARIES})

gz_install_library(gazebo_sdf_interface)
gz_install_includes("sdf/interface" ${headers})
