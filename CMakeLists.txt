
CMAKE_MINIMUM_REQUIRED( VERSION 2.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
ENDIF(COMMAND CMAKE_POLICY)

PROJECT (Gazebo)
SET (GAZEBO_VERSION 0.9)
STRING (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

SET (gazeboserver_sources_desc "List of server sources" 
     CACHE INTERNAL "Gazebo server sources list description" FORCE)
SET (gazeboserver_include_dirs_desc "List of server include directories" 
     CACHE INTERNAL "Include dirs description")
SET (gazeboserver_link_dirs_desc "List of server link directories" 
     CACHE INTERNAL "Link dirs description" FORCE)
SET (gazeboserver_link_libs_desc "List of server link libraries" 
     CACHE INTERNAL "Link libs" FORCE)

SET (OGRE_VERSION 1.6.0 CACHE INTERNAL "Ogre Version Requirement")
SET (ODE_VERSION 0.10.1 CACHE INTERNAL "ODE Version Requirement")

SET (gazebo_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")
set (gazebo_cflags "-ggdb -g2 -Wall -fPIC" CACHE INTERNAL "Gazebo CFlags")

ADD_SUBDIRECTORY(libgazebo)
ADD_SUBDIRECTORY(server)
#ADD_SUBDIRECTORY(player)

