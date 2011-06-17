project(gazebo_tools)

cmake_minimum_required( VERSION 2.4 FATAL_ERROR )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0004 NEW)
endif(COMMAND cmake_policy)

include (FindPkgConfig)

#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(PKG_CFGS REQUIRED gazebo_transport)
#endif()

message (STATUS "TINY: ${TINY_XML_INCLUDE_DIRS}")
include_directories(${TINY_XML_INCLUDE_DIRS})

add_library( convert  joint.cpp link.cpp model.cpp parser.cpp)
target_link_libraries(convert ${TINY_XML_LIBRARIES})

add_executable( check  check_gdf.cpp )
target_link_libraries(check ${TINY_XML_LIBRARIES} convert)


