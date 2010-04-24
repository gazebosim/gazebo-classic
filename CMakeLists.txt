CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
ENDIF(COMMAND CMAKE_POLICY)

project (Gazebo)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (GAZEBO_MAJOR_VERSION 0)
set (GAZEBO_MINOR_VERSION 10)
set (GAZEBO_PATCH_VERSION 0)

set (GAZEBO_VERSION ${GAZEBO_MAJOR_VERSION}.${GAZEBO_MINOR_VERSION}.${GAZEBO_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${GAZEBO_VERSION}")

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set (BUILD_GAZEBO ON CACHE INTERNAL "Build Gazebo" FORCE)

set (gazeboserver_sources_desc "List of server sources" 
     CACHE INTERNAL "Gazebo server sources list description" FORCE)
set (gazeboserver_headers_desc "List of server headers" 
     CACHE INTERNAL "Gazebo server headers list description" FORCE)
set (gazeboserver_include_dirs_desc "List of server include directories" 
     CACHE INTERNAL "Include dirs description")
set (gazeboserver_link_dirs_desc "List of server link directories" 
     CACHE INTERNAL "Link dirs description" FORCE)
set (gazeboserver_link_libs_desc "List of server link libraries" 
     CACHE INTERNAL "Link libs description" FORCE)
set (gazeboserver_ldflags_desc "List of server ldflags" 
     CACHE INTERNAL "LDFlags description" FORCE)
set (gazeboserver_cflags_desc "List of server cflags" 
     CACHE INTERNAL "Server cflags description" FORCE)
set (gazebosensor_sources_desc "List of sensor sources" 
     CACHE INTERNAL "Gazebo sensor sources list description" FORCE)
set (gazebocontroller_sources_desc "List of controller sources" 
     CACHE INTERNAL "Gazebo controller sources list description" FORCE)

set (bullet_link_libs_desc "List of bullet link libraries" 
     CACHE INTERNAL "Bullet link libs description" FORCE)

set (MIN_OGRE_VERSION 1.6.3 CACHE INTERNAL "Ogre version requirement" FORCE)

set (FREEIMAGE_MAJOR_VERSION 3 CACHE INTERNAL "FreeImage major version requirement" FORCE)
set (FREEIMAGE_MINOR_VERSION 9 CACHE INTERNAL "FreeImage minor version requirement" FORCE)
set (FREEIMAGE_VERSION ${FREEIMAGE_MAJOR_VERSION}.${FREEIMAGE_MINOR_VERSION}.0 CACHE INTERNAL "FreeImage version requirement" FORCE)
set (ODE_VERSION 0.10.1 CACHE INTERNAL "ODE version requirement" FORCE)
set (MIN_BOOST_VERSION 1.35.0 CACHE INTERNAL "Boost min version requirement" FORCE)

set (ENABLE_BINDINGS OFF CACHE BOOL "Enable libgazebo bindings")

set (gazebo_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

INCLUDE (${gazebo_cmake_dir}/DissectVersion.cmake)

#####################################
# Build type cflags
SET (CMAKE_C_FLAGS_RELEASE " -O2 -DNDEBUG -Wall -fPIC " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_C_FLAGS_DEBUG " -ggdb  -Wall -fPIC " CACHE INTERNAL "C Flags for debug" FORCE)
SET (CMAKE_C_FLAGS_PROFILE " -ggdb -pg -Wall -fPIC " CACHE INTERNAL "C Flags for profile" FORCE)

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

SET (build_errors "" CACHE INTERNAL "build errors" FORCE)

MESSAGE (STATUS "\n\n====== Finding 3rd Party Packages ======")
INCLUDE (${gazebo_cmake_dir}/SearchForStuff.cmake)
MESSAGE (STATUS "----------------------------------------\n")

if ( build_errors )
  message(STATUS "!!! BUILD ERRORS !!!")
  message(STATUS "These errors must be resolved before compiling")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach (msg ${build_errors})
  message(STATUS "!!! END BUILD ERRORS !!!\n")
else (build_errors)
  ########################################
  # Write the config.h file
  configure_file (${PROJECT_SOURCE_DIR}/config.h.in ${PROJECT_BINARY_DIR}/config.h)

  configure_file (${PROJECT_SOURCE_DIR}/gazebo.h.in ${PROJECT_BINARY_DIR}/gazebo.h)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gazebo.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo)
  
  ########################################
  # Make the gazebo rc file
  configure_file(${CMAKE_SOURCE_DIR}/cmake/gazeborc.cmake ${PROJECT_BINARY_DIR}/.gazeborc @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc DESTINATION $ENV{HOME}/)
 
  include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
  
  message (STATUS "Cmake C Flags:${CMAKE_C_FLAGS}")
  
  if (BUILD_GAZEBO)
    include (libgazebo)

    if (use_internal_assimp)
      add_subdirectory(3rd_party)
    endif (use_internal_assimp)

    add_subdirectory(libgazebo)
    add_subdirectory(server)
    add_subdirectory(worlds)
    add_subdirectory(tools)
    add_subdirectory(Media)
    
    if (INCLUDE_PLAYER)
      add_subdirectory(player)
      add_subdirectory(player_cfgs)
    endif (INCLUDE_PLAYER)
    
    if (INCLUDE_WEBGAZEBO)
      add_subdirectory(webgazebo)
    endif (INCLUDE_WEBGAZEBO)
    
    message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
    message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
  endif (BUILD_GAZEBO)
  
  # Cpack stuff
  include (InstallRequiredSystemLibraries)
  set (CPACK_PACKAGE_NAME "${PROJECT_NAME}")
  set (CPACK_PACKAGE_VERSION_MAJOR "${GAZEBO_MAJOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_MINOR "${GAZEBO_MINOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_PATCH "${GAZEBO_PATCH_VERSION}")
  set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "Gazebo robot simulator")
  set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWER}-${GAZEBO_VERSION}")
  set (CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWER}-${GAZEBO_VERSION}")
  set (CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/COPYING")
  set (CPACK_GENERATOR "TBZ2")
  set (CPACK_SOURCE_GENERATOR "TBZ2")
  list (APPEND CPACK_SOURCE_IGNORE_FILES "/.svn/;.swp$;/build/")
  include (CPack)

  message(STATUS "Configuration successful. Type make to compile gazebo")

endif(build_errors)

GAZEBOSERVER_RESET_LISTS()
