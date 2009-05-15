project(graphics3d)

cmake_minimum_required( VERSION 2.4 FATAL_ERROR )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0004 NEW)
endif(COMMAND cmake_policy)

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(PKG_CFGS REQUIRED libgazebo)
  set (INCLUDE_DIRS ${PKG_CFGS_INCLUDE_DIRS})
  set (LINK_DIRS  ${PKG_CFGS_LIBRARY_DIRS})
  set (LINK_LIBS ${PKG_CFGS_LINK_LIBS} ${PKG_CFGS_LIBRARIES})
endif (PKG_CONFIG_FOUND)

include_directories( ${INCLUDE_DIRS} )
link_directories( ${LINK_DIRS} )


include_directories(.)

set (sources graphics3d.cc )

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-ggdb -g2 -Wall")

add_executable(graphics3d ${sources})

target_link_libraries(graphics3d ${LINK_LIBS} )
