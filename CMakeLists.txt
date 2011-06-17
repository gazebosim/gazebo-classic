cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (Gazebo)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (GAZEBO_VERSION_NAME "hydrogen")
set (GAZEBO_MAJOR_VERSION 1)
set (GAZEBO_MINOR_VERSION 0)
set (GAZEBO_PATCH_VERSION 0)

set (GAZEBO_VERSION ${GAZEBO_MAJOR_VERSION}.${GAZEBO_MINOR_VERSION}.${GAZEBO_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${GAZEBO_VERSION}")

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set (BUILD_GAZEBO ON CACHE INTERNAL "Build Gazebo" FORCE)

set (INCLUDE_INSTALL_DIR "include/gazebo")
set (LIB_INSTALL_DIR "lib")
set (BIN_INSTALL_DIR "bin")


# with -fPIC
if(UNIX AND NOT WIN32)
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif(UNIX AND NOT WIN32)


set (MIN_OGRE_VERSION 1.7.1 CACHE INTERNAL "Ogre version requirement" FORCE)
set (MIN_ODE_VERSION 0.11.1 CACHE INTERNAL "ODE version requirement" FORCE)
set (MIN_BOOST_VERSION 1.40.0 CACHE INTERNAL "Boost min version requirement" FORCE)

set (FREEIMAGE_MAJOR_VERSION 3 CACHE INTERNAL "FreeImage major version requirement" FORCE)
set (FREEIMAGE_MINOR_VERSION 9 CACHE INTERNAL "FreeImage minor version requirement" FORCE)
set (MIN_FREEIMAGE_VERSION ${FREEIMAGE_MAJOR_VERSION}.${FREEIMAGE_MINOR_VERSION}.0 CACHE INTERNAL "FreeImage version requirement" FORCE)


set (gazebo_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

include (${gazebo_cmake_dir}/DissectVersion.cmake)

#####################################
# Build type cflags
set (CMAKE_C_FLAGS_RELEASE " -O0 -DNDEBUG -Wall -pedantic -Wno-long-long -Wno-unused-value -fPIC " CACHE INTERNAL "C Flags for release" FORCE)
set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

set (CMAKE_C_FLAGS_DEBUG " -ggdb -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-value -fPIC " CACHE INTERNAL "C Flags for debug" FORCE)
set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})

set (CMAKE_C_FLAGS_PROFILE " -ggdb -pg -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-value -fPIC " CACHE INTERNAL "C Flags for profile" FORCE)
set (CMAKE_CXX_FLAGS_PROFILE ${CMAKE_C_FLAGS_PROFILE})

set (CMAKE_C_FLAGS_CHECK " -ggdb -Wall -Wextra -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default -Wfloat-equal -Wshadow -pedantic -Wno-long-long -Wno-unused-value -fPIC" CACHE INTERNAL "C Flags for static code checking" FORCE)
set (CMAKE_CXX_FLAGS_CHECK ${CMAKE_C_FLAGS_CHECK})

#####################################
# Build type link flags
set (CMAKE_LINK_FLAGS_RELEASE " -fPIC" CACHE INTERNAL "Link flags fro release" FORCE)
set (CMAKE_LINK_FLAGS_DEBUG " -fPIC" CACHE INTERNAL "Link flags for debug" FORCE)
set (CMAKE_LINK_FLAGS_PROFILE " -fPIC" CACHE INTERNAL "Link flags for profile" FORCE)
set (CMAKE_LINK_FLAGS_CHECK " -fPIC" CACHE INTERNAL "Link flags for static code checking" FORCE)

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "release" CACHE STRING 
    "Choose the type of build, options are: debug release profile check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

#####################################
# Set all the global build flags
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")

set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
include (${gazebo_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

if ( build_warnings )
  message(STATUS "BUILD WARNINGS")
  foreach (msg ${build_warnings})
    message(STATUS ${msg})
  endforeach (msg ${build_warnings})
  message(STATUS "END BUILD WARNINGS\n")
endif (build_warnings)

if ( build_errors )
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach (msg ${build_errors})
  message(STATUS "END BUILD ERRORS\n")
else (build_errors)

  ########################################
  # Write the config.h file
  configure_file (${gazebo_cmake_dir}/gazebo_config.h.in ${PROJECT_BINARY_DIR}/gazebo_config.h)
  INSTALL_INCLUDES(${PROJECT_BINARY_DIR}/gazebo_config.h "")

  configure_file (${gazebo_cmake_dir}/gazebo.h.in ${PROJECT_BINARY_DIR}/gazebo.h)
  INSTALL_INCLUDES(${CMAKE_CURRENT_BINARY_DIR}/gazebo.h "")
  
  ########################################
  # Make the gazebo rc file
  configure_file(${CMAKE_SOURCE_DIR}/cmake/gazeborc.cmake ${PROJECT_BINARY_DIR}/.gazeborc @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc DESTINATION $ENV{HOME}/)
 
  include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
  
  message (STATUS "C Flags:${CMAKE_CXX_FLAGS}")
  message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
 
  if (BUILD_GAZEBO)
    add_subdirectory(src)
    add_subdirectory(sdf)
    add_subdirectory(Media)
    add_subdirectory(test)
    add_subdirectory(tools)
   
    #if (INCLUDE_PLAYER)
    #  add_subdirectory(player)
    #  add_subdirectory(player_cfgs)
    #endif (INCLUDE_PLAYER)
  endif (BUILD_GAZEBO)

  ########################################
  # Make the package config files
  configure_file(${CMAKE_SOURCE_DIR}/cmake/gazebo_transport_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/cmake/gazebo_transport.pc @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/gazebo_transport.pc DESTINATION lib/pkgconfig COMPONENT pkgconfig)

  configure_file(${CMAKE_SOURCE_DIR}/cmake/gazebo_server_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/cmake/gazebo_server.pc @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/gazebo_server.pc DESTINATION lib/pkgconfig COMPONENT pkgconfig)

  configure_file(${CMAKE_SOURCE_DIR}/cmake/gazebo_client_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/cmake/gazebo_client.pc @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/gazebo_client.pc DESTINATION lib/pkgconfig COMPONENT pkgconfig)

  ########################################
  # Package Creation: 
  include (${gazebo_cmake_dir}/gazebo_cpack.cmake)
  set (CPACK_PACKAGE_VERSION "${GAZEBO_VERSION}")
  set (CPACK_PACKAGE_VERSION_MAJOR "${GAZEBO_MAJOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_MINOR "${GAZEBO_MINOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_PATCH "${GAZEBO_PATCH_VERSION}")

  if (CPACK_GENERATOR)
    message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

    configure_file("${gazebo_cmake_dir}/cpack_options.cmake.in" ${GAZEBO_CPACK_CFG_FILE} @ONLY)
    set(CPACK_PROJECT_CONFIG_FILE ${GAZEBO_CPACK_CFG_FILE})
    include (CPack)
  endif()

  message(STATUS "Configuration successful. Type make to compile gazebo")
endif(build_errors)

