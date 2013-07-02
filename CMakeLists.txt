cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (Gazebo)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set (GAZEBO_VERSION_NAME "lithium")
set (GAZEBO_MAJOR_VERSION 1)
set (GAZEBO_MINOR_VERSION 9)
# The patch version may have been bumped for prerelease purposes; be sure to
# check gazebo-release/ubuntu/debian/changelog@default to determine what the
# next patch version should be for a regular release.
set (GAZEBO_PATCH_VERSION 0)

set (GAZEBO_VERSION ${GAZEBO_MAJOR_VERSION}.${GAZEBO_MINOR_VERSION})
set (GAZEBO_VERSION_FULL ${GAZEBO_MAJOR_VERSION}.${GAZEBO_MINOR_VERSION}.${GAZEBO_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${GAZEBO_VERSION_FULL}")

set (gazebo_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")

########################################
# Package Creation: 
include (${gazebo_cmake_dir}/gazebo_cpack.cmake)
set (CPACK_PACKAGE_VERSION "${GAZEBO_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${GAZEBO_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${GAZEBO_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${GAZEBO_PATCH_VERSION}")

if (CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

  configure_file("${gazebo_cmake_dir}/cpack_options.cmake.in" ${GAZEBO_CPACK_CFG_FILE} @ONLY)
  set(CPACK_PROJECT_CONFIG_FILE ${GAZEBO_CPACK_CFG_FILE})
  include (CPack)
endif()

# If we're configuring only to package source, stop here
if (PACKAGE_SOURCE_ONLY)
  message(WARNING "Configuration was done in PACKAGE_SOURCE_ONLY mode.  You can build a tarball (make package_source), but nothing else.")
  return()
endif()

# Documentation
add_subdirectory(doc)

# If we're configuring only to build docs, stop here
if (DOC_ONLY)
  message(WARNING "Configuration was done in DOC_ONLY mode.  You can build documentation (make doc), but nothing else.")
  return()
endif()

enable_testing()

# with -fPIC
if(UNIX AND NOT WIN32)
  set (CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "Install Prefix")
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif()

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)


# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()


set (INCLUDE_INSTALL_DIR "include/gazebo-${GAZEBO_VERSION}/gazebo") 
set (LIB_INSTALL_DIR "lib" CACHE STRING "Installation directory for libraries (relative to CMAKE_INSTALL_PREFIX)") 
set (BIN_INSTALL_DIR "bin" CACHE STRING "Installation directory for binaries (relative to CMAKE_INSTALL_PREFIX)") 


set (USE_FULL_RPATH OFF CACHE BOOL "Set to true to enable full rpath")

if (USE_FULL_RPATH)
  # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH  FALSE)
  
  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
  
  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  
  # the RPATH to be used when installing, but only if its not a system directory
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
  endif("${isSystemDir}" STREQUAL "-1")
endif()

set (BUILD_GAZEBO ON CACHE INTERNAL "Build Gazebo" FORCE)
set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)


set (MIN_OGRE_VERSION 1.7.1 CACHE INTERNAL "Ogre version requirement" FORCE)
set (MIN_BOOST_VERSION 1.40.0 CACHE INTERNAL "Boost min version requirement" FORCE)

set (FREEIMAGE_MAJOR_VERSION 3 CACHE INTERNAL "FreeImage major version requirement" FORCE)
set (FREEIMAGE_MINOR_VERSION 9 CACHE INTERNAL "FreeImage minor version requirement" FORCE)
set (MIN_FREEIMAGE_VERSION ${FREEIMAGE_MAJOR_VERSION}.${FREEIMAGE_MINOR_VERSION}.0 CACHE INTERNAL "FreeImage version requirement" FORCE)

include (${gazebo_cmake_dir}/DissectVersion.cmake)

#####################################
# Build type cflags
set (CMAKE_C_FLAGS_ALL " -Wall -Wextra -Wno-long-long -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wfloat-equal -Wshadow -Winit-self -Wswitch-default -Wmissing-include-dirs -pedantic" CACHE INTERNAL "General C Flags")


#####################################
# Build type link flags
set (CMAKE_LINK_FLAGS_RELEASE " " CACHE INTERNAL "Link flags fro release" FORCE)
set (CMAKE_LINK_FLAGS_RELWITHDEBINFO " " CACHE INTERNAL "Link flags for release with debug support" FORCE)
set (CMAKE_LINK_FLAGS_DEBUG " " CACHE INTERNAL "Link flags for debug" FORCE)
set (CMAKE_LINK_FLAGS_PROFILE " -pg" CACHE INTERNAL "Link flags for profile" FORCE)
set (CMAKE_LINK_FLAGS_CHECK " --coverage" CACHE INTERNAL "Link flags for static code checking" FORCE)

#####################################
# Check for low memory version to use in some tests
if(NOT DEFINED USE_LOW_MEMORY_TESTS)
  set (USE_LOW_MEMORY_TESTS FALSE)
  message (STATUS "High memory tests: enabled")
else()
  set (USE_LOW_MEMORY_TEST TRUE)
  message (STATUS "High memory tests: disabled, low memory versions will be used")
endif()

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
include (${gazebo_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

if (SSE2_FOUND)
  set (CMAKE_C_FLAGS_ALL "${CMAKE_C_FLAGS_ALL} -msse -msse2 -mfpmath=sse")
endif()
if (SSE3_FOUND)
  set (CMAKE_C_FLAGS_ALL "${CMAKE_C_FLAGS_ALL} -msse3")
endif()
if (SSSE3_FOUND)
  set (CMAKE_C_FLAGS_ALL "${CMAKE_C_FLAGS_ALL} -mssse3")
endif()

if (ENABLE_SSE4)
  message(STATUS "\nSSE4 will be enabled if system supports it.\n")
  if (SSE4_1_FOUND)
    set (CMAKE_C_FLAGS_ALL "${CMAKE_C_FLAGS_ALL} -msse4.1")
  endif()
  if (SSE4_2_FOUND)
    set (CMAKE_C_FLAGS_ALL "${CMAKE_C_FLAGS_ALL} -msse4.2")
  endif()
else()
  message(STATUS "\nSSE4 disabled.\n")
endif()

# Check for DRI capable Display
include (${gazebo_cmake_dir}/CheckDRIDisplay.cmake)

set (CMAKE_C_FLAGS_RELEASE " -s -O3 -DNDEBUG ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release" FORCE)
set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

set (CMAKE_C_FLAGS_RELWITHDEBINFO " -g -O2 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release with debug support" FORCE)
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})

set (CMAKE_C_FLAGS_DEBUG " -ggdb3 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for debug" FORCE)
set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})

set (CMAKE_C_FLAGS_PROFILE " -fno-omit-frame-pointer -g -pg ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for profile" FORCE)
set (CMAKE_CXX_FLAGS_PROFILE ${CMAKE_C_FLAGS_PROFILE})

set (CMAKE_C_FLAGS_CHECK " -g -O0 -Wformat=2 --coverage -fno-inline ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for static code checking" FORCE)
set (CMAKE_CXX_FLAGS_CHECK "${CMAKE_C_FLAGS_CHECK} -fno-elide-constructors -fno-default-inline -fno-implicit-inline-templates")

#####################################
MESSAGE(STATUS "Checking gazebo build type")
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release" CACHE STRING 
        "Choose the type of build, options are: Debug Release RelWithDebInfo Profile Check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
# TODO: still convert to uppercase to keep backwards compatibility with 
# uppercase old supported and deprecated modes
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

#####################################
# Set all the global build flags
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")

set (BUILD_TYPE_PROFILE FALSE)
set (BUILD_TYPE_RELEASE FALSE)
set (BUILD_TYPE_RELWITHDEBINFO FALSE)
set (BUILD_TYPE_DEBUG FALSE)

if ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "PROFILE")
  set (BUILD_TYPE_PROFILE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set (BUILD_TYPE_RELEASE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set (BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set (BUILD_TYPE_DEBUG TRUE)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug Release RelWithDebInfo Profile Check")
endif()

# TODO: remove this check when transtition period ends
# Check if build type was sent in old uppercase format
if ("${CMAKE_BUILD_TYPE}" STREQUAL "${CMAKE_BUILD_TYPE_UPPERCASE}")
    build_warning("CMAKE_BUILD_TYPE was specified in uppercase. This will stop to be supported soon. Valid types: Debug Release RelWithDebInfo Profile Check")
endif ()

message (STATUS "\n\n====== Configuring 3rd Party Packages ======")
add_subdirectory(deps)
message (STATUS "----------------------------------------\n")

#################################################
# OS Specific initialization
if (UNIX)
  gz_setup_unix()
else (WIN32)
  gz_setup_windows()
else (APPLE)
  gz_setup_apple()
endif()

#################################################
# Print warnings and errors
if ( build_warnings )
  message(STATUS "BUILD WARNINGS")
  foreach (msg ${build_warnings})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD WARNINGS\n")
endif (build_warnings)

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

if (build_errors)
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build. Please see the BUILD ERRORS above.")
else (build_errors)

  ########################################
  # Write the config.h file
  configure_file (${gazebo_cmake_dir}/gazebo_config.h.in ${PROJECT_BINARY_DIR}/gazebo/gazebo_config.h)
  gz_install_includes("" ${PROJECT_BINARY_DIR}/gazebo/gazebo_config.h)

  configure_file(${CMAKE_SOURCE_DIR}/cmake/setup.sh.in ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/)
  configure_file(${CMAKE_SOURCE_DIR}/cmake/setup-unversioned.sh.in ${PROJECT_BINARY_DIR}/unversioned/setup.sh @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/unversioned/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/)
 
  include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
  
  message (STATUS "C Flags:${CMAKE_CXX_FLAGS}")
  message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
 
  if (BUILD_GAZEBO)
    set(TEST_TYPE "UNIT")
    add_subdirectory(gazebo)
    add_subdirectory(media)
    add_subdirectory(test)
    add_subdirectory(tools)
    add_subdirectory(plugins)
    add_subdirectory(interfaces)
    add_subdirectory(rml)
  endif (BUILD_GAZEBO)

  ########################################
  # Make the package config files
  set (pkgconfig_files gazebo_ode gazebo_transport gazebo)

  foreach (pkgconfig ${pkgconfig_files})
    configure_file(${CMAKE_SOURCE_DIR}/cmake/pkgconfig/${pkgconfig}.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}.pc DESTINATION ${LIB_INSTALL_DIR}/pkgconfig COMPONENT pkgconfig)
  endforeach()

  ########################################
  # Make the cmake config files
  set(PKG_NAME ${PROJECT_NAME_UPPER})
  if (HAVE_RML)
    set(PKG_LIBRARIES gazebo_physics gazebo_common gazebo_math gazebo_rendering gazebo_gui gazebo_ode gazebo_msgs gazebo_transport)
  else()
    set(PKG_LIBRARIES gazebo_physics gazebo_common gazebo_math gazebo_rendering gazebo_gui gazebo_ode gazebo_msgs gazebo_transport gazebo_sdf_interface)
  endif()

  set(PKG_DEPENDS Boost Protobuf RML)
  set(cmake_conf_file "cmake/gazebo-config.cmake")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}" @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file} DESTINATION share/${PROJECT_NAME_LOWER}/cmake/ COMPONENT cmake)


  ########################################
  # If present, load platform-specific build hooks.  This system is used,
  # for example, by the Ubuntu overlay (in the gazebo-release repo), to
  # arrange for installation of Ubuntu-specific application-launching
  # configuration.
  if (EXISTS ${PROJECT_SOURCE_DIR}/cmake/packager-hooks/CMakeLists.txt)
    message(STATUS "Loading packager build hooks from cmake/packager-hooks")
    add_subdirectory(cmake/packager-hooks)
  endif()

  message(STATUS "Configuration successful. Type make to compile gazebo")
endif(build_errors)
