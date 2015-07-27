cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (Gazebo)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set (GAZEBO_MAJOR_VERSION 6)
set (GAZEBO_MINOR_VERSION 0)
# The patch version may have been bumped for prerelease purposes; be sure to
# check gazebo-release/ubuntu/debian/changelog@default to determine what the
# next patch version should be for a regular release.
set (GAZEBO_PATCH_VERSION 10)

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

# Configure documentation uploader
configure_file("${CMAKE_SOURCE_DIR}/cmake/upload_doc.sh.in"
  ${CMAKE_BINARY_DIR}/upload_doc.sh @ONLY)

# If we're configuring only to build docs, stop here
if (DOC_ONLY)
  message(WARNING "Configuration was done in DOC_ONLY mode.  You can build documentation (make doc), but nothing else.")
  return()
endif()

enable_testing()

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

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


set (INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/gazebo-${GAZEBO_VERSION}/gazebo")
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set (BIN_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})

set (BUILD_GAZEBO ON CACHE INTERNAL "Build Gazebo" FORCE)
set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)


set (MIN_OGRE_VERSION 1.7.4 CACHE INTERNAL "Ogre version requirement" FORCE)
set (MIN_BOOST_VERSION 1.40.0 CACHE INTERNAL "Boost min version requirement" FORCE)

set (FREEIMAGE_MAJOR_VERSION 3 CACHE INTERNAL "FreeImage major version requirement" FORCE)
set (FREEIMAGE_MINOR_VERSION 9 CACHE INTERNAL "FreeImage minor version requirement" FORCE)
set (MIN_FREEIMAGE_VERSION ${FREEIMAGE_MAJOR_VERSION}.${FREEIMAGE_MINOR_VERSION}.0 CACHE INTERNAL "FreeImage version requirement" FORCE)

include (${gazebo_cmake_dir}/DissectVersion.cmake)

#####################################
# Check for low memory version to use in some tests
if(NOT DEFINED USE_LOW_MEMORY_TESTS)
  set (USE_LOW_MEMORY_TESTS FALSE)
  message (STATUS "High memory tests: enabled")
else()
  set (USE_LOW_MEMORY_TEST TRUE)
  message (STATUS "High memory tests: disabled, low memory versions will be used")
endif()

######################################
# Enable screen tests by default
if(NOT DEFINED ENABLE_SCREEN_TESTS)
  set(ENABLE_SCREEN_TESTS TRUE)
endif()

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
include (${gazebo_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

#####################################
# Define some variables that are going to be used in two places:
# 1. In CMake code to pass preprocessor definitions to certain source files
# (especially in common/CMakeLists.txt).
# 2. In the generation of cmake/setup.sh from cmake/setup.sh.in
set(GAZEBO_DEFAULT_MASTER_HOST localhost)
set(GAZEBO_DEFAULT_MASTER_PORT 11345)
set(GAZEBO_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins)
set(GAZEBO_MODEL_PATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo-${GAZEBO_VERSION}/models)
set(GAZEBO_RESOURCE_PATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo-${GAZEBO_VERSION}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo_models)
set(GAZEBO_MODEL_DATABASE_URI http://gazebosim.org/models)
set(OGRE_RESOURCE_PATH ${OGRE_PLUGINDIR})
# Seems that OGRE_PLUGINDIR can end in a newline, which will cause problems when
# we pass it to the compiler later.
string(REPLACE "\n" "" OGRE_RESOURCE_PATH ${OGRE_RESOURCE_PATH})


# Check for DRI capable Display
include (${gazebo_cmake_dir}/CheckDRIDisplay.cmake)

#####################################
MESSAGE(STATUS "Checking gazebo build type")
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo Profile Check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
# TODO: still convert to uppercase to keep backwards compatibility with
# uppercase old supported and deprecated modes
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

if (NOT DEFINED HDF5_INSTRUMENT)
  set (HDF5_INSTRUMENT FALSE)
endif()

set (BUILD_TYPE_PROFILE FALSE)
set (BUILD_TYPE_RELEASE FALSE)
set (BUILD_TYPE_RELWITHDEBINFO FALSE)
set (BUILD_TYPE_DEBUG FALSE)
set (BUILD_TYPE_COVERAGE FALSE)

if ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "PROFILE")
  set (BUILD_TYPE_PROFILE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set (BUILD_TYPE_RELEASE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set (BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set (BUILD_TYPE_DEBUG TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "COVERAGE")
  # Disable DART for now due to eigen problem (issue #1160)
  set (HAVE_DART FALSE)
  set (BUILD_TYPE_COVERAGE TRUE)
  include (${gazebo_cmake_dir}/CodeCoverage.cmake)
  setup_target_for_coverage(coverage coverage)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug Release RelWithDebInfo Profile Check")
endif()

#####################################
# Handle CFlags
# USE_UPSTREAM_CFLAGS (default TRUE)
if(NOT DEFINED USE_UPSTREAM_CFLAGS)
    set (USE_UPSTREAM_CFLAGS True)
endif()

unset (CMAKE_C_FLAGS_ALL CACHE)

# USE_HOST_CFLAGS (default TRUE)
# Will check building host machine for proper cflags
if(NOT DEFINED USE_HOST_CFLAGS OR USE_HOST_CFLAGS)
  message(STATUS "Enable host CFlags")
  include (${gazebo_cmake_dir}/HostCFlags.cmake)
endif()

# Will use predefined gazebo developers cflags
# this needs to be called after HostCFlags 
if(USE_UPSTREAM_CFLAGS)
  # use gazebo own set of flags
  unset (CMAKE_CXX_FLAGS CACHE)
  message(STATUS "Enable upstream CFlags")
  include(${gazebo_cmake_dir}/DefaultCFlags.cmake)
endif()

# Check if warning options are avaliable for the compiler and return WARNING_CXX_FLAGS variable
filter_valid_compiler_flags(-Wall -Wextra -Wno-long-long -Wno-unused-value -Wno-unused-value
                               -Wno-unused-value -Wno-unused-value -Wfloat-equal -Wshadow
			       -Winit-self -Wswitch-default -Wmissing-include-dirs -pedantic)

# Check and add visibility hidden by default. Only in UNIX
# Windows and MacosX does not handled properly the hidden compilation
if (UNIX AND NOT APPLE)
  filter_valid_compiler_flags(-fvisibility=hidden -fvisibility-inlines-hidden)
endif()

if (MSVC)
  # Unable to be filtered flags (failing due to limitations in filter_valid_compiler_warnings)
  # Handling exceptions rightly
  set(UNFILTERED_FLAGS "/EHsc")
endif()

# Visual Studio enables c++11 support by default
if (NOT MSVC)
  set(UNFILTERED_FLAGS "-std=c++11")
endif()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VALID_CXX_FLAGS} ${UNFILTERED_FLAGS}")

#################################################
# OS Specific initialization
if (UNIX)
  gz_setup_unix()
else (WIN32)
  gz_setup_windows()
endif()

if(APPLE)
  gz_setup_apple()
endif()

# Main includes for compilation
include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
include_directories(SYSTEM
  ${IGNITION-MATH_INCLUDE_DIRS}
)

#################################################
# Configure 3rd Party Packages after OS Specific initialization
message (STATUS "\n\n====== Configuring 3rd Party Packages ======")
add_subdirectory(deps)
message (STATUS "----------------------------------------\n")

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
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo-${GAZEBO_VERSION}/)
  # Also install the setup.sh in an unversioned location
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gazebo/)


  if (DEFINED CMAKE_CXX_FLAGS)
    message (STATUS "Custom CFlags:${CMAKE_CXX_FLAGS}")
  else()
    message (STATUS "Use default CFlags")
  endif()
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
    add_subdirectory(worlds)
    add_subdirectory(models)
  endif (BUILD_GAZEBO)

  ########################################
  # Make the package config files
  set (pkgconfig_files gazebo_ode gazebo_transport gazebo)

  # set boost pkgconfig cflags
  set (Boost_PKGCONFIG_CFLAGS ${Boost_INCLUDE_DIRS})
  if (NOT "${Boost_PKGCONFIG_CFLAGS}" STREQUAL "")
    set (Boost_PKGCONFIG_CFLAGS "-I${Boost_PKGCONFIG_CFLAGS}")
  endif (NOT "${Boost_PKGCONFIG_CFLAGS}" STREQUAL "")
  string (REPLACE ";" " -I" Boost_PKGCONFIG_CFLAGS "${Boost_PKGCONFIG_CFLAGS}")

  # set boost pkgconfig libs
  set (Boost_PKGCONFIG_LIBS ${Boost_LIBRARY_DIRS})
  if (NOT "${Boost_PKGCONFIG_LIBS}" STREQUAL "")
    set (Boost_PKGCONFIG_LIBS "-L${Boost_PKGCONFIG_LIBS}")
  endif(NOT "${Boost_PKGCONFIG_LIBS}" STREQUAL "")
  string (REPLACE ";" " -L" Boost_PKGCONFIG_LIBS "${Boost_PKGCONFIG_LIBS}")

  foreach (b ${Boost_LIBRARIES})
    get_filename_component(bname ${b} NAME_WE)
    # Prefix always -l
    set (bname "-l${bname}")
    # Remove the prefix lib (not always present, like in pthread)
    string (REPLACE "lib" "" bname "${bname}")
    set (Boost_PKGCONFIG_LIBS "${Boost_PKGCONFIG_LIBS} ${bname}")
  endforeach(b)

  foreach (pkgconfig ${pkgconfig_files})
    configure_file(${CMAKE_SOURCE_DIR}/cmake/pkgconfig/${pkgconfig}.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}.pc DESTINATION ${LIB_INSTALL_DIR}/pkgconfig COMPONENT pkgconfig)
  endforeach()

  ########################################
  # Make the cmake config files
  set(PKG_NAME ${PROJECT_NAME_UPPER})

  # Order is important, if A depends on B, please add B after A.
  # The list should have at the very end the libraries 
  # without internal interdependencies
  set(PKG_LIBRARIES
    gazebo
    gazebo_client
    gazebo_gui_building
    gazebo_gui_viewers
    gazebo_gui_model
    gazebo_gui
    gazebo_sensors
    gazebo_rendering
    gazebo_selection_buffer
  )

  # "Second level" deps. Depends on a variety of first level deps
  if (HAVE_DART)
    set(PKG_LIBRARIES ${PKG_LIBRARIES} gazebo_physics_dart)
  endif()
  if (HAVE_BULLET)
    set(PKG_LIBRARIES ${PKG_LIBRARIES} gazebo_physics_bullet)
  endif()
  if (HAVE_SIMBODY)
    set(PKG_LIBRARIES ${PKG_LIBRARIES} gazebo_physics_simbody)
  endif()

  if (INCLUDE_PLAYER)
    set(PKG_LIBRARIES ${PKG_LIBRARIES} gazebo_player)
  endif()

  set(PKG_LIBRARIES ${PKG_LIBRARIES}
    gazebo_physics_ode
    gazebo_physics
    gazebo_ode
  )

  # "First level" deps, mostly depends on math and/or common
  if (${OGRE_VERSION} GREATER 1.7.4)
    set(PKG_LIBRARIES ${PKG_LIBRARIES} gazebo_rendering_deferred)
  endif()

  set(PKG_LIBRARIES ${PKG_LIBRARIES}
    gazebo_transport
    gazebo_msgs
    gazebo_util
    gazebo_common)

  # No other internal dependencies:
  set(PKG_LIBRARIES ${PKG_LIBRARIES}
    gazebo_skyx
    gazebo_gimpact
    gazebo_opcode
    gazebo_opende_ou
    gazebo_math
  )

  if (NOT CCD_FOUND)
    set(PKG_LIBRARIES ${PKG_LIBRARIES} gazebo_ccd)
  endif()

  set(PKG_DEPENDS Boost Protobuf SDFormat OGRE)

  set(cmake_conf_file "cmake/gazebo-config.cmake")
  set(cmake_conf_version_file "cmake/gazebo-config-version.cmake")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}" @ONLY)
  # Use write_basic_package_version_file to generate a ConfigVersion file that
  # allow users of gazebo to specify the API or version to depend on
  # TODO: keep this instruction until deprecate Ubuntu/Precise and update with
  # https://github.com/Kitware/CMake/blob/v2.8.8/Modules/CMakePackageConfigHelpers.cmake
  include(WriteBasicConfigVersionFile)
  write_basic_config_version_file(
      ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
      VERSION "${GAZEBO_VERSION_FULL}"
      COMPATIBILITY SameMajorVersion)
  install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}
            ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
          DESTINATION
	    ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME_LOWER}/
	  COMPONENT cmake)
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
