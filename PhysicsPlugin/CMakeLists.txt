cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include(GNUInstallDirs)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


# todo: This is just an example. This should be cleaned up and moved to a more permanent location.
add_library(PhysicsPluginExample SHARED PhysicsPluginExample.c)
target_link_libraries(PhysicsPluginExample ${GAZEBO_libraries})

# todo: where to install this
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
install (TARGETS PhysicsPluginExample DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/)

# below should not be needed
# target_link_libraries(gazebo_physics PhysicsPluginExample)
