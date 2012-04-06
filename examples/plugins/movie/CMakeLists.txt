cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})

add_library(movie_pr2 SHARED pr2.cc)
target_link_libraries(movie_pr2 ${GAZEBO_libraries})
install(TARGETS movie_pr2 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_VERSION}/plugins)

add_library(movie SHARED movie.cc)
target_link_libraries(movie ${GAZEBO_libraries} ${OGRE_libraries})
install(TARGETS movie DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_VERSION}/plugins)
