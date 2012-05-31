include_directories(SYSTEM ${XML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIRS})
add_definitions(${TINYXML_CFLAGS})

set (sources Param.cc SDF.cc parser.cc parser_deprecated.cc)
set (headers SDF.hh Param.hh parser.hh parser_deprecated.hh)

gz_add_library(gazebo_sdf_interface ${sources})
target_link_libraries(gazebo_sdf_interface gazebo_common ${TINYXML_LIBRARIES} ${XML_LIBRARIES} ${Boost_LIBRARIES})

INSTALL_LIBRARY(gazebo_sdf_interface)
INSTALL_INCLUDES("sdf/interface" ${headers})
