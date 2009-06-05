CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
ENDIF(COMMAND CMAKE_POLICY)

PROJECT (Gazebo)
STRING (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
SET (GAZEBO_VERSION 0.9.0)
MESSAGE (STATUS "${PROJECT_NAME} version ${GAZEBO_VERSION}")

SET (BUILD_GAZEBO ON CACHE INTERNAL "Build Gazebo" FORCE)

SET (gazeboserver_sources_desc "List of server sources" 
     CACHE INTERNAL "Gazebo server sources list description" FORCE)
SET (gazeboserver_headers_desc "List of server headers" 
     CACHE INTERNAL "Gazebo server headers list description" FORCE)
SET (gazeboserver_include_dirs_desc "List of server include directories" 
     CACHE INTERNAL "Include dirs description")
SET (gazeboserver_link_dirs_desc "List of server link directories" 
     CACHE INTERNAL "Link dirs description" FORCE)
SET (gazeboserver_link_libs_desc "List of server link libraries" 
     CACHE INTERNAL "Link libs description" FORCE)
SET (gazeboserver_cflags_desc "List of server cflags" 
     CACHE INTERNAL "Server cflags description" FORCE)
SET (gazebosensor_sources_desc "List of sensor sources" 
     CACHE INTERNAL "Gazebo sensor sources list description" FORCE)
SET (gazebocontroller_sources_desc "List of controller sources" 
     CACHE INTERNAL "Gazebo controller sources list description" FORCE)

SET (OGRE_VERSION 1.6.1 CACHE INTERNAL "Ogre version requirement" FORCE)

SET (FREEIMAGE_MAJOR_VERSION 3 CACHE INTERNAL "FreeImage major version requirement" FORCE)
SET (FREEIMAGE_MINOR_VERSION 10 CACHE INTERNAL "FreeImage minor version requirement" FORCE)
SET (FREEIMAGE_VERSION ${FREEIMAGE_MAJOR_VERSION}.${FREEIMAGE_MINOR_VERSION}.0 CACHE INTERNAL "FreeImage version requirement" FORCE)
SET (ODE_VERSION 0.10.1 CACHE INTERNAL "ODE version requirement" FORCE)
SET (MIN_BOOST_VERSION 1.35.0 CACHE INTERNAL "Boost min version requirement" FORCE)

SET (gazebo_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

#####################################
# Build type cflags
SET (CMAKE_C_FLAGS_RELEASE " -O3 -DNDEBUG -Wall " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_C_FLAGS_DEBUG " -ggdb " CACHE INTERNAL "C Flags fro debug" FORCE)
SET (CMAKE_C_FLAGS_PROFILE " -ggdb -pg " CACHE INTERNAL "C Flags for profile" FORCE)

#####################################
# Build type link flags
SET (CMAKE_LINK_FLAGS_RELEASE "" CACHE INTERNAL "Link flags fro release" FORCE)
SET (CMAKE_LINK_FLAGS_DEBUG "" CACHE INTERNAL "Link flags for debug" FORCE)
SET (CMAKE_LINK_FLAGS_PROFILE "" CACHE INTERNAL "Link flags for profile" FORCE)

#####################################
# Set the default build type
IF (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE "release" CACHE STRING 
    "Choose the type of build, options are: debug release profile" FORCE)
ENDIF (NOT CMAKE_BUILD_TYPE)
STRING(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

MESSAGE (STATUS "====== Finding 3rd Party Packages ======")
INCLUDE (${gazebo_cmake_dir}/SearchForStuff.cmake)
MESSAGE (STATUS "----------------------------------------")

########################################
# Write the config.h file
CONFIGURE_FILE (${PROJECT_SOURCE_DIR}/config.h.in ${PROJECT_BINARY_DIR}/config.h)

########################################
# Make the gazebo rc file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/gazeborc.cmake ${PROJECT_BINARY_DIR}/.gazeborc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc DESTINATION $ENV{HOME}/)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})

IF (BUILD_GAZEBO)
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
  
  MESSAGE (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  MESSAGE (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
ENDIF (BUILD_GAZEBO)

GAZEBOSERVER_RESET_LISTS()
