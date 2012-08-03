#-------------------------------------------------------------------
# This file is part of the CMake build system for SkyX
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

#######################################################################
# This file takes care of configuring SkyX to build with the settings
# given in CMake. It creates the necessary config.h file and will
# also prepare package files for pkg-config and CMake.
#######################################################################

# No static build for the moment

# dynamic or static build
if (SKYX_STATIC)
  set(SKYX_LIB_TYPE STATIC)
else ()
  set(SKYX_LIB_TYPE SHARED)
endif ()

# Create the pkg-config package files on Unix systems
if (UNIX)
  set(SKYX_LIB_SUFFIX "")
  set(SKYX_PLUGIN_PREFIX "")
  set(SKYX_PLUGIN_EXT ".so")
  if (SKYX_STATIC)
    set(SKYX_LIB_SUFFIX "${SKYX_LIB_SUFFIX}Static")
    set(SKYX_PLUGIN_PREFIX "lib")
    set(SKYX_PLUGIN_EXT ".a")
  endif ()
  string(TOLOWER "${CMAKE_BUILD_TYPE}" SKYX_BUILD_TYPE)
  if (SKYX_BUILD_TYPE STREQUAL "debug")
    set(SKYX_LIB_SUFFIX "${SKYX_LIB_SUFFIX}_d")
  endif ()

  set(SKYX_ADDITIONAL_LIBS "")
  set(SKYX_CFLAGS "")
  set(SKYX_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
  configure_file(${SKYX_TEMPLATES_DIR}/SKYX.pc.in ${SKYX_BINARY_DIR}/pkgconfig/SKYX${SKYX_LIB_SUFFIX}.pc @ONLY)

  install(FILES ${SKYX_BINARY_DIR}/pkgconfig/SKYX${SKYX_LIB_SUFFIX}.pc DESTINATION lib/pkgconfig)
endif ()

if (MSVC)
  # Enable intrinsics on MSVC in debug mode
  # Not actually necessary in release mode since /O2 implies /Oi but can't easily add this per build type?
  add_definitions(/Oi)
endif (MSVC)

