project(gazebo_tools)

cmake_minimum_required( VERSION 2.4 FATAL_ERROR )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0004 NEW)
endif(COMMAND cmake_policy)

include (FindPkgConfig)

add_subdirectory(xml)

#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(PKG_CFGS REQUIRED gazebo_transport)
#endif()

#include_directories(${PKG_CFGS_INCLUDE_DIRS})

#add_executable( convert convert.cc Converter.cc)

#target_link_libraries(convert ${PKG_CFGS_LIBRARIES})
