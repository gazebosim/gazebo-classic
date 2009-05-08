CMAKE_MINIMUM_REQUIRED( VERSION 2.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
ENDIF(COMMAND CMAKE_POLICY)

PROJECT (Gazebo)
STRING (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
SET (GAZEBO_VERSION 0.9.0)
MESSAGE (STATUS "${PROJECT_NAME} version ${GAZEBO_VERSION}")

SET (gazeboserver_sources_desc "List of server sources" 
     CACHE INTERNAL "Gazebo server sources list description" FORCE)
SET (gazeboserver_include_dirs_desc "List of server include directories" 
     CACHE INTERNAL "Include dirs description")
SET (gazeboserver_link_dirs_desc "List of server link directories" 
     CACHE INTERNAL "Link dirs description" FORCE)
SET (gazeboserver_link_libs_desc "List of server link libraries" 
     CACHE INTERNAL "Link libs description" FORCE)
SET (gazeboserver_cflags_desc "List of server cflags" 
     CACHE INTERNAL "Server cflags description" FORCE)

SET (OGRE_VERSION 1.6.0 CACHE INTERNAL "Ogre Version Requirement")
SET (ODE_VERSION 0.10.1 CACHE INTERNAL "ODE Version Requirement")

SET (gazebo_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

# Build type cflags
SET (CMAKE_C_FLAGS_RELEASE " -O3 -DNDEBUG -fPIC -Wall " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_C_FLAGS_DEBUG " -ggdb -fPIC " CACHE INTERNAL "C Flags fro debug" FORCE)
SET (CMAKE_C_FLAGS_PROFILE " -ggdb -pg -fPIC " CACHE INTERNAL "C Flags for profile" FORCE)

# Build type link flags
SET (CMAKE_LINK_FLAGS_RELEASE "" CACHE INTERNAL "Link flags fro release" FORCE)
SET (CMAKE_LINK_FLAGS_DEBUG "" CACHE INTERNAL "Link flags for debug" FORCE)
SET (CMAKE_LINK_FLAGS_PROFILE "" CACHE INTERNAL "Link flags for profile" FORCE)

FIND_LIBRARY(PROFILER "profiler")
IF (PROFILER)
  SET (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -lprofiler" 
       CACHE INTERNAL "Link flags for profile" FORCE)
ENDIF (PROFILER)

FIND_LIBRARY(TCMALLOC "tcmalloc")
IF (TCMALLOC)
  SET (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -ltcmalloc" 
       CACHE INTERNAL "Link flags for profile" FORCE)
ENDIF (TCMALLOC)

IF (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE "release" CACHE STRING 
    "Choose the type of build, options are: debug release profile" FORCE)
ENDIF (NOT CMAKE_BUILD_TYPE)

STRING(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

MESSAGE (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")


MESSAGE (STATUS "====== Finding 3rd Party Packages ======")
INCLUDE (${gazebo_cmake_dir}/SearchForStuff.cmake)
MESSAGE (STATUS "----------------------------------------")

ADD_SUBDIRECTORY(libgazebo)
ADD_SUBDIRECTORY(server)
ADD_SUBDIRECTORY(worlds)
ADD_SUBDIRECTORY(Media)

IF (INCLUDE_PLAYER)
  ADD_SUBDIRECTORY(player)
ENDIF (INCLUDE_PLAYER)

IF (INCLUDE_WEBGAZEBO)
  Add_SUBDIRECTORY(webgazebo)
ENDIF (INCLUDE_WEBGAZEBO)
