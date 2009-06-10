cmake_minimum_required( VERSION 2.4.6 FATAL_ERROR )

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (ogretest)

include (FindPkgConfig)
include (FindFLTK)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE)
endif (PKG_CONFIG_FOUND)

include_directories(${OGRE_INCLUDE_DIRS})

set (sources main.cc GLWindow.cc)

add_executable(ogretest ${sources})

target_link_libraries(ogretest ${OGRE_LIBRARIES} ${FLTK_LIBRARIES})
