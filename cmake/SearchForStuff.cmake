include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (CheckCXXSourceCompiles)

include (${gazebo_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)
include (${gazebo_cmake_dir}/FindFreeimage.cmake)

execute_process(COMMAND pkg-config --modversion protobuf
  OUTPUT_VARIABLE PROTOBUF_VERSION
  RESULT_VARIABLE protobuf_modversion_failed)

########################################
# 1. can not use BUILD_TYPE_PROFILE is defined after include this module
# 2. TODO: TOUPPER is a hack until we fix the build system to support standard build names
if (CMAKE_BUILD_TYPE)
  string(TOUPPER ${CMAKE_BUILD_TYPE} TMP_CMAKE_BUILD_TYPE)
  if ("${TMP_CMAKE_BUILD_TYPE}" STREQUAL "PROFILE")
    include (${gazebo_cmake_dir}/FindGooglePerfTools.cmake)
    if (GOOGLE_PERFTOOLS_FOUND)
      message(STATUS "Include google-perftools")
    else()
      BUILD_ERROR("Need google/heap-profiler.h (libgoogle-perftools-dev) tools to compile in Profile mode")
    endif()
  endif()
endif()

########################################
if (PROTOBUF_VERSION LESS 2.3.0)
  BUILD_ERROR("Incorrect version: Gazebo requires protobuf version 2.3.0 or greater")
endif()

########################################
# The Google Protobuf library for message generation + serialization
find_package(Protobuf REQUIRED)
if (NOT PROTOBUF_FOUND)
  BUILD_ERROR ("Missing: Google Protobuf (libprotobuf-dev)")
endif()
if (NOT PROTOBUF_PROTOC_EXECUTABLE)
  BUILD_ERROR ("Missing: Google Protobuf Compiler (protobuf-compiler)")
endif()
if (NOT PROTOBUF_PROTOC_LIBRARY)
  BUILD_ERROR ("Missing: Google Protobuf Compiler Library (libprotoc-dev)")
endif()

########################################
include (FindOpenGL)
if (NOT OPENGL_FOUND)
  BUILD_ERROR ("Missing: OpenGL")
else ()
 APPEND_TO_CACHED_LIST(gazeboserver_include_dirs
                       ${gazeboserver_include_dirs_desc}
                       ${OPENGL_INCLUDE_DIR})
 APPEND_TO_CACHED_LIST(gazeboserver_link_libs
                       ${gazeboserver_link_libs_desc}
                       ${OPENGL_LIBRARIES})
endif ()

########################################
include (FindOpenAL)
if (NOT OPENAL_FOUND)
  BUILD_WARNING ("OpenAL not found, audio support will be disabled.")
  set (HAVE_OPENAL OFF CACHE BOOL "HAVE OpenAL" FORCE)
else ()
  set (HAVE_OPENAL ON CACHE BOOL "HAVE OpenAL" FORCE)
endif ()

########################################
# Find packages
if (PKG_CONFIG_FOUND)
  pkg_check_modules(CURL libcurl)
  if (NOT CURL_FOUND)
    BUILD_ERROR ("Missing: libcurl. Required for connection to model database.")
  endif()

  pkg_check_modules(PROFILER libprofiler)
  if (PROFILER_FOUND)
    set (CMAKE_LINK_FLAGS_PROFILE "-Wl,--no-as-needed -lprofiler -Wl,--as-needed ${CMAKE_LINK_FLAGS_PROFILE}" CACHE INTERNAL "Link flags for profile")
  else ()
    find_library(PROFILER profiler)
    if (PROFILER)
      message (STATUS "Looking for libprofiler - found")
      set (CMAKE_LINK_FLAGS_PROFILE "-Wl,--no-as-needed -lprofiler -Wl,--as-needed ${CMAKE_LINK_FLAGS_PROFILE}" CACHE INTERNAL "Link flags for profile")
    else()
      message (STATUS "Looking for libprofiler - not found")
    endif()
  endif()

  pkg_check_modules(TCMALLOC libtcmalloc)
  if (TCMALLOC_FOUND)
    set (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -Wl,--no-as-needed -ltcmalloc -Wl,--no-as-needed"
      CACHE INTERNAL "Link flags for profile" FORCE)
  else ()
    find_library(TCMALLOC tcmalloc)
    if (TCMALLOC)
      message (STATUS "Looking for libtcmalloc - found")
      set (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -ltcmalloc"
        CACHE INTERNAL "Link flags for profile" FORCE)
    else ()
      message (STATUS "Looking for libtcmalloc - not found")
    endif()
  endif ()

  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI_OGRE CEGUI-OGRE)
  if (NOT CEGUI_FOUND)
    BUILD_WARNING ("CEGUI not found, opengl GUI will be disabled.")
    set (HAVE_CEGUI OFF CACHE BOOL "HAVE CEGUI" FORCE)
  else()
    message (STATUS "Looking for CEGUI, found")
    if (NOT CEGUI_OGRE_FOUND)
      BUILD_WARNING ("CEGUI-OGRE not found, opengl GUI will be disabled.")
      set (HAVE_CEGUI OFF CACHE BOOL "HAVE CEGUI" FORCE)
    else()
      set (HAVE_CEGUI ON CACHE BOOL "HAVE CEGUI")
      set (CEGUI_LIBRARIES "CEGUIBase;CEGUIOgreRenderer")
      message (STATUS "Looking for CEGUI-OGRE, found")
    endif()
  endif()

  #################################################
  # Find Simbody
  set(SimTK_INSTALL_DIR ${SimTK_INSTALL_PREFIX})
  #list(APPEND CMAKE_MODULE_PATH ${SimTK_INSTALL_PREFIX}/share/cmake)
  find_package(Simbody)
  if (SIMBODY_FOUND)
    set (HAVE_SIMBODY TRUE)
  else()
    BUILD_WARNING ("Simbody not found, for simbody physics engine option, please install libsimbody-dev.")
    set (HAVE_SIMBODY FALSE)
  endif()

  #################################################
  # Find DART
  find_package(DARTCore 4.1 QUIET)
  if (DARTCore_FOUND)
    message (STATUS "Looking for DARTCore - found")
    set (HAVE_DART TRUE)
  else()
    message (STATUS "Looking for DARTCore - not found")
    BUILD_WARNING ("DART not found, for dart physics engine option, please install libdart-core4-dev.")
    set (HAVE_DART FALSE)
  endif()

  #################################################
  # Find tinyxml. Only debian distributions package tinyxml with a pkg-config
  # Use pkg_check_modules and fallback to manual detection (needed, at least, for MacOS)
  pkg_check_modules(tinyxml tinyxml)
  if (NOT tinyxml_FOUND)
      find_path (tinyxml_INCLUDE_DIRS tinyxml.h ${tinyxml_INCLUDE_DIRS} ENV CPATH)
      find_library(tinyxml_LIBRARIES NAMES tinyxml)
      set (tinyxml_FAIL False)
      if (NOT tinyxml_INCLUDE_DIRS)
        message (STATUS "Looking for tinyxml headers - not found")
        set (tinyxml_FAIL True)
      endif()
      if (NOT tinyxml_LIBRARIES)
        message (STATUS "Looking for tinyxml library - not found")
        set (tinyxml_FAIL True)
      endif()
  endif()

  if (tinyxml_FAIL)
    message (STATUS "Looking for tinyxml.h - not found")
    BUILD_ERROR("Missing: tinyxml")
  endif()

  #################################################
  # Find libtar.
  find_path (libtar_INCLUDE_DIRS libtar.h)
  find_library(libtar_LIBRARIES tar)
  set (LIBTAR_FOUND True)

  if (NOT libtar_INCLUDE_DIRS)
    message (STATUS "Looking for libtar.h - not found")
    set (LIBTAR_FOUND False)
  else ()
    message (STATUS "Looking for libtar.h - found")
    include_directories(${libtar_INCLUDE_DIRS})
  endif ()
  if (NOT libtar_LIBRARIES)
    message (STATUS "Looking for libtar.so - not found")
    set (LIBTAR_FOUND False)
  else ()
    message (STATUS "Looking for libtar.so - found")
  endif ()

  if (NOT LIBTAR_FOUND)
     BUILD_ERROR("Missing: libtar")
  endif()

  #################################################
  # Find TBB
  pkg_check_modules(TBB tbb)
  if (NOT TBB_FOUND)
    message(STATUS "TBB not found, attempting to detect manually")

    find_library(tbb_library tbb ENV LD_LIBRARY_PATH)
    if (tbb_library)
      set(TBB_FOUND true)
      set(TBB_LIBRARIES ${tbb_library})
    else (tbb_library)
      BUILD_ERROR ("Missing: TBB - Threading Building Blocks")
    endif(tbb_library)
  endif (NOT TBB_FOUND)

  #################################################
  # Find OGRE
  execute_process(COMMAND pkg-config --modversion OGRE
                  OUTPUT_VARIABLE OGRE_VERSION)
  string(REPLACE "\n" "" OGRE_VERSION ${OGRE_VERSION})

  pkg_check_modules(OGRE-RTShaderSystem
                    OGRE-RTShaderSystem>=${MIN_OGRE_VERSION})

  if (OGRE-RTShaderSystem_FOUND)
    set(ogre_ldflags ${OGRE-RTShaderSystem_LDFLAGS})
    set(ogre_include_dirs ${OGRE-RTShaderSystem_INCLUDE_DIRS})
    set(ogre_libraries ${OGRE-RTShaderSystem_LIBRARIES})
    set(ogre_library_dirs ${OGRE-RTShaderSystem_LIBRARY_DIRS})
    set(ogre_cflags ${OGRE-RTShaderSystem_CFLAGS})

    set (INCLUDE_RTSHADER ON CACHE BOOL "Enable GPU shaders")
  else ()
    set (INCLUDE_RTSHADER OFF CACHE BOOL "Enable GPU shaders")
  endif ()

  pkg_check_modules(OGRE OGRE>=${MIN_OGRE_VERSION})
  # There are some runtime problems to solve with ogre-1.9.
  # Please read gazebo issues: 994, 995
  if (NOT OGRE_FOUND)
    BUILD_ERROR("Missing: Ogre3d version >=${MIN_OGRE_VERSION}(http://www.orge3d.org)")
  else ()
    set(ogre_ldflags ${ogre_ldflags} ${OGRE_LDFLAGS})
    set(ogre_include_dirs ${ogre_include_dirs} ${OGRE_INCLUDE_DIRS})
    set(ogre_libraries ${ogre_libraries};${OGRE_LIBRARIES})
    set(ogre_library_dirs ${ogre_library_dirs} ${OGRE_LIBRARY_DIRS})
    set(ogre_cflags ${ogre_cflags} ${OGRE_CFLAGS})
  endif ()

  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  if (OGRE-Terrain_FOUND)
    set(ogre_ldflags ${ogre_ldflags} ${OGRE-Terrain_LDFLAGS})
    set(ogre_include_dirs ${ogre_include_dirs} ${OGRE-Terrain_INCLUDE_DIRS})
    set(ogre_libraries ${ogre_libraries};${OGRE-Terrain_LIBRARIES})
    set(ogre_library_dirs ${ogre_library_dirs} ${OGRE-Terrain_LIBRARY_DIRS})
    set(ogre_cflags ${ogre_cflags} ${OGRE-Terrain_CFLAGS})
  endif()

  pkg_check_modules(OGRE-Overlay OGRE-Overlay)
  if (OGRE-Overlay_FOUND)
    set(ogre_ldflags ${ogre_ldflags} ${OGRE-Overlay_LDFLAGS})
    set(ogre_include_dirs ${ogre_include_dirs} ${OGRE-Overlay_INCLUDE_DIRS})
    set(ogre_libraries ${ogre_libraries};${OGRE-Overlay_LIBRARIES})
    set(ogre_library_dirs ${ogre_library_dirs} ${OGRE-Overlay_LIBRARY_DIRS})
    set(ogre_cflags ${ogre_cflags} ${OGRE-Overlay_CFLAGS})
  endif()


  set (OGRE_INCLUDE_DIRS ${ogre_include_dirs}
       CACHE INTERNAL "Ogre include path")

  # Also find OGRE's plugin directory, which is provided in its .pc file as the
  # `plugindir` variable.  We have to call pkg-config manually to get it.
  execute_process(COMMAND pkg-config --variable=plugindir OGRE
                  OUTPUT_VARIABLE _pkgconfig_invoke_result
                  RESULT_VARIABLE _pkgconfig_failed)
  if(_pkgconfig_failed)
    BUILD_WARNING ("Failed to find OGRE's plugin directory.  The build will succeed, but gazebo will likely fail to run.")
  else()
    # This variable will be substituted into cmake/setup.sh.in
    set (OGRE_PLUGINDIR ${_pkgconfig_invoke_result})
  endif()

  ########################################
  # Check and find libccd (if needed)
  pkg_check_modules(CCD ccd>=1.4)
  if (NOT CCD_FOUND)
    message(STATUS "Using internal copy of libccd")
    set(CCD_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/deps/libccd/include")
    set(CCD_LIBRARIES gazebo_ccd)
  endif()

  ########################################
  # Find OpenAL
  # pkg_check_modules(OAL openal)
  # if (NOT OAL_FOUND)
  #   BUILD_WARNING ("Openal not found. Audio capabilities will be disabled.")
  #   set (HAVE_OPENAL FALSE)
  # else (NOT OAL_FOUND)
  #   set (HAVE_OPENAL TRUE)
  # endif ()

  ########################################
  # Find libswscale format
  pkg_check_modules(libswscale libswscale)
  if (NOT libswscale_FOUND)
    BUILD_WARNING ("libswscale not found. Audio-video capabilities will be disabled.")
  else()
    include_directories(${libswscale_INCLUDE_DIRS})
    link_directories(${libswscale_LIBRARY_DIRS})
  endif ()

  ########################################
  # Find AV format
  pkg_check_modules(libavformat libavformat)
  if (NOT libavformat_FOUND)
    BUILD_WARNING ("libavformat not found. Audio-video capabilities will be disabled.")
  else()
    include_directories(${libavformat_INCLUDE_DIRS})
    link_directories(${libavformat_LIBRARY_DIRS})
  endif ()

  ########################################
  # Find avcodec
  pkg_check_modules(libavcodec libavcodec)
  if (NOT libavcodec_FOUND)
    BUILD_WARNING ("libavcodec not found. Audio-video capabilities will be disabled.")
  else()
    include_directories(${libavcodec_INCLUDE_DIRS})
    link_directories(${libavcodec_LIBRARY_DIRS})
  endif ()

  ########################################
  # Find avutil
  pkg_check_modules(libavutil libavutil)
  if (NOT libavutil_FOUND)
    BUILD_WARNING ("libavutil not found. Audio-video capabilities will be disabled.")
  endif ()


  if (libavutil_FOUND AND libavformat_FOUND AND libavcodec_FOUND AND libswscale_FOUND)
    set (HAVE_FFMPEG TRUE)
  else ()
    set (HAVE_FFMPEG FALSE)
  endif ()

  ########################################
  # Find Player
  pkg_check_modules(PLAYER playercore>=3.0 playerc++ playerwkb)
  if (NOT PLAYER_FOUND)
    set (INCLUDE_PLAYER OFF CACHE BOOL "Build gazebo plugin for player")
    BUILD_WARNING ("Player not found, gazebo plugin for player will not be built.")
  else (NOT PLAYER_FOUND)
    set (INCLUDE_PLAYER ON CACHE BOOL "Build gazebo plugin for player")
    set (PLAYER_INCLUDE_DIRS ${PLAYER_INCLUDE_DIRS} CACHE INTERNAL
         "Player include directory")
    set (PLAYER_LINK_DIRS ${PLAYER_LINK_DIRS} CACHE INTERNAL
         "Player link directory")
    set (PLAYER_LINK_LIBS ${PLAYER_LIBRARIES} CACHE INTERNAL
         "Player libraries")
  endif ()

  ########################################
  # Find GNU Triangulation Surface Library
  pkg_check_modules(gts gts)
  if (gts_FOUND)
    message (STATUS "Looking for GTS - found")
    set (HAVE_GTS TRUE)
  else ()
    set (HAVE_GTS FALSE)
    BUILD_WARNING ("GNU Triangulation Surface library not found - Gazebo will not have CSG support.")
  endif ()

  #################################################
  # Find bullet
  # First and preferred option is to look for bullet standard pkgconfig,
  # so check it first. if it is not present, check for the OSRF
  # custom bullet2.82.pc file
  pkg_check_modules(BULLET bullet>=2.82)
  if (NOT BULLET_FOUND)
     pkg_check_modules(BULLET bullet2.82>=2.82)
  endif()

  if (BULLET_FOUND)
    set (HAVE_BULLET TRUE)
    add_definitions( -DLIBBULLET_VERSION=${BULLET_VERSION} )
  else()
    set (HAVE_BULLET FALSE)
    add_definitions( -DLIBBULLET_VERSION=0.0 )
    BUILD_WARNING ("Bullet > 2.82 not found, for bullet physics engine option, please install libbullet2.82-dev.")
  endif()

else (PKG_CONFIG_FOUND)
  set (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
  BUILD_ERROR ("Error: pkg-config not found")
endif ()

########################################
# Find SDFormat
find_package(SDFormat 2.0.1)
if (NOT SDFormat_FOUND)
  message (STATUS "Looking for SDFormat - not found")
  BUILD_ERROR ("Missing: SDF version >=2.0.1. Required for reading and writing SDF files.")
else()
  message (STATUS "Looking for SDFormat - found")
endif()

########################################
# Find QT
find_package (Qt4)
if (NOT QT4_FOUND)
  BUILD_ERROR("Missing: Qt4")
endif()

########################################
# Find Boost, if not specified manually
include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED thread signals system filesystem program_options regex iostreams date_time)

if (NOT Boost_FOUND)
  set (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
  BUILD_ERROR ("Boost not found. Please install thread signals system filesystem program_options regex date_time boost version ${MIN_BOOST_VERSION} or higher.")
endif()

########################################
# Find libdl
find_path(libdl_include_dir dlfcn.h /usr/include /usr/local/include)
if (NOT libdl_include_dir)
  message (STATUS "Looking for dlfcn.h - not found")
  BUILD_ERROR ("Missing libdl: Required for plugins.")
  set (libdl_include_dir /usr/include)
else (NOT libdl_include_dir)
  message (STATUS "Looking for dlfcn.h - found")
endif ()

find_library(libdl_library dl /usr/lib /usr/local/lib)
if (NOT libdl_library)
  message (STATUS "Looking for libdl - not found")
  BUILD_ERROR ("Missing libdl: Required for plugins.")
else (NOT libdl_library)
  message (STATUS "Looking for libdl - found")
endif ()

########################################
# Find gdal
include (FindGDAL)
if (NOT GDAL_FOUND)
  message (STATUS "Looking for libgdal - not found")
  BUILD_WARNING ("GDAL not found, Digital elevation terrains support will be disabled.")
  set (HAVE_GDAL OFF CACHE BOOL "HAVE GDAL" FORCE)
else ()
  message (STATUS "Looking for libgdal - found")
  set (HAVE_GDAL ON CACHE BOOL "HAVE GDAL" FORCE)
endif ()

########################################
# Find libusb
pkg_check_modules(libusb-1.0 libusb-1.0)
if (NOT libusb-1.0_FOUND)
  BUILD_WARNING ("libusb-1.0 not found. USB peripherals support will be disabled.")
  set (HAVE_USB OFF CACHE BOOL "HAVE USB" FORCE)
else()
  message (STATUS "Looking for libusb-1.0 - found. USB peripherals support enabled.")
  set (HAVE_USB ON CACHE BOOL "HAVE USB" FORCE)
  include_directories(${libusb-1.0_INCLUDE_DIRS})
  link_directories(${libusb-1.0_LIBRARY_DIRS})
endif ()

#################################################
# Find Oculus SDK.
pkg_check_modules(OculusVR OculusVR)

if (HAVE_USB AND OculusVR_FOUND)
  message (STATUS "Oculus Rift support enabled.")
  set (HAVE_OCULUS ON CACHE BOOL "HAVE OCULUS" FORCE)
  include_directories(SYSTEM ${OculusVR_INCLUDE_DIRS})
  link_directories(${OculusVR_LIBRARY_DIRS})
else ()
  BUILD_WARNING ("Oculus Rift support will be disabled.")
  set (HAVE_OCULUS OFF CACHE BOOL "HAVE OCULUS" FORCE)
endif()

########################################
# Include man pages stuff
include (${gazebo_cmake_dir}/Ronn2Man.cmake)
include (${gazebo_cmake_dir}/Man.cmake)
add_manpage_target()

########################################
# Find QWT (QT graphing library)
#find_path(QWT_INCLUDE_DIR NAMES qwt.h PATHS
#  /usr/include
#  /usr/local/include
#  "$ENV{LIB_DIR}/include"
#  "$ENV{INCLUDE}"
#  PATH_SUFFIXES qwt-qt4 qwt qwt5
#  )
#
#find_library(QWT_LIBRARY NAMES qwt qwt6 qwt5 PATHS
#  /usr/lib
#  /usr/local/lib
#  "$ENV{LIB_DIR}/lib"
#  "$ENV{LIB}/lib"
#  )
#
#if (QWT_INCLUDE_DIR AND QWT_LIBRARY)
#  set(HAVE_QWT TRUE)
#endif (QWT_INCLUDE_DIR AND QWT_LIBRARY)
#
#if (HAVE_QWT)
#  if (NOT QWT_FIND_QUIETLY)
#    message(STATUS "Found Qwt: ${QWT_LIBRARY}")
#  endif (NOT QWT_FIND_QUIETLY)
#else ()
#  if (QWT_FIND_REQUIRED)
#    BUILD_WARNING ("Could not find libqwt-dev. Plotting features will be disabled.")
#  endif (QWT_FIND_REQUIRED)
#endif ()
