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

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  set (GZ_PROTOBUF_LIBRARY ${PROTOBUF_LIBRARY_DEBUG})
  set (GZ_PROTOBUF_PROTOC_LIBRARY ${PROTOBUF_PROTOC_LIBRARY_DEBUG})
else()
  set (GZ_PROTOBUF_LIBRARY ${PROTOBUF_LIBRARY})
  set (GZ_PROTOBUF_PROTOC_LIBRARY ${PROTOBUF_PROTOC_LIBRARY})
endif()

########################################
include (FindOpenGL)
if (NOT OPENGL_FOUND)
  BUILD_ERROR ("Missing: OpenGL")
  set (HAVE_OPENGL FALSE)
else ()
 if (OPENGL_INCLUDE_DIR)
   APPEND_TO_CACHED_LIST(gazeboserver_include_dirs
                         ${gazeboserver_include_dirs_desc}
                         ${OPENGL_INCLUDE_DIR})
   set (HAVE_OPENGL TRUE)
   add_definitions(-DHAVE_OPENGL)
 endif()
 if (OPENGL_LIBRARIES)
   APPEND_TO_CACHED_LIST(gazeboserver_link_libs
                         ${gazeboserver_link_libs_desc}
                         ${OPENGL_LIBRARIES})
 endif()
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
include (FindHDF5)
find_package(HDF5)

if (NOT HDF5_FOUND)
  BUILD_WARNING("HDF5 not found")
else ()
  message(STATUS "HDF5 Found")
endif ()

########################################
# Find packages

# In Visual Studio we use configure.bat to trick all path cmake
# variables so let's consider that as a replacement for pkgconfig
if (MSVC)
  set (PKG_CONFIG_FOUND TRUE)
endif()

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

  #################################################
  # Find Simbody
  set(SimTK_INSTALL_DIR ${SimTK_INSTALL_PREFIX})
  #list(APPEND CMAKE_MODULE_PATH ${SimTK_INSTALL_PREFIX}/share/cmake)
  find_package(Simbody)
  if (Simbody_FOUND)
    message (STATUS "Looking for Simbody - found")
    set (HAVE_SIMBODY TRUE)
  else()
    message (STATUS "Looking for Simbody - not found")
    BUILD_WARNING ("Simbody not found, for simbody physics engine option, please install libsimbody-dev.")
    set (HAVE_SIMBODY FALSE)
  endif()

  #################################################
  # Find DART
  find_package(DARTCore 5.1.1 QUIET)
  if (DARTCore_FOUND)
    message (STATUS "Looking for DARTCore - ${DARTCore_VERSION} found")
    set (HAVE_DART TRUE)
  else()
    message (STATUS "Looking for DARTCore - not found")
    BUILD_WARNING ("DART not found, for dart physics engine option, please install libdart-core5-dev.")
    set (HAVE_DART FALSE)
  endif()

  #################################################
  # Find tinyxml. Only debian distributions package tinyxml with a pkg-config
  # Use pkg_check_modules and fallback to manual detection
  # (needed, at least, for MacOS)

  # Use system installation on UNIX and Apple, and internal copy on Windows
  if (UNIX OR APPLE)
    message (STATUS "Using system tinyxml.")
    set (USE_EXTERNAL_TINYXML True)
  elseif(WIN32)
    message (STATUS "Using internal tinyxml.")
    set (USE_EXTERNAL_TINYXML False)
    add_definitions(-DTIXML_USE_STL)
  else()
    message (STATUS "Unknown platform, unable to configure tinyxml.")
    BUILD_ERROR("Unknown platform")
  endif()

  if (USE_EXTERNAL_TINYXML)
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
  else()
    # Needed in WIN32 since in UNIX the flag is added in the code installed
    message (STATUS "Skipping search for tinyxml")
    set (tinyxml_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/deps/win/tinyxml")
    set (tinyxml_LIBRARIES "")
    set (tinyxml_LIBRARY_DIRS "")
  endif()

  #################################################
  # Find tinyxml2. Only debian distributions package tinyxml with a pkg-config
  # Use pkg_check_modules and fallback to manual detection
  # (needed, at least, for MacOS)

  # Use system installation on UNIX and Apple, and internal copy on Windows
  if (UNIX OR APPLE)
    message (STATUS "Using system tinyxml2.")
    set (USE_EXTERNAL_TINYXML2 True)
  elseif(WIN32)
    message (STATUS "Using internal tinyxml2.")
    set (USE_EXTERNAL_TINYXML2 False)
  else()
    message (STATUS "Unknown platform, unable to configure tinyxml2.")
    BUILD_ERROR("Unknown platform")
  endif()

  if (USE_EXTERNAL_TINYXML2)
    pkg_check_modules(tinyxml2 tinyxml2)
    if (NOT tinyxml2_FOUND)
        find_path (tinyxml2_INCLUDE_DIRS tinyxml2.h ${tinyxml2_INCLUDE_DIRS} ENV CPATH)
        find_library(tinyxml2_LIBRARIES NAMES tinyxml2)
        set (tinyxml2_FAIL False)
        if (NOT tinyxml2_INCLUDE_DIRS)
          message (STATUS "Looking for tinyxml2 headers - not found")
          set (tinyxml2_FAIL True)
        endif()
        if (NOT tinyxml2_LIBRARIES)
          message (STATUS "Looking for tinyxml2 library - not found")
          set (tinyxml2_FAIL True)
        endif()
        if (NOT tinyxml2_LIBRARY_DIRS)
          message (STATUS "Looking for tinyxml2 library dirs - not found")
          set (tinyxml2_FAIL True)
        endif()
    endif()

    if (tinyxml2_FAIL)
      message (STATUS "Looking for tinyxml2.h - not found")
      BUILD_ERROR("Missing: tinyxml2")
    else()
      include_directories(${tinyxml2_INCLUDE_DIRS})
      link_directories(${tinyxml2_LIBRARY_DIRS})
    endif()
  else()
    # Needed in WIN32 since in UNIX the flag is added in the code installed
    message (STATUS "Skipping search for tinyxml2")
    set (tinyxml2_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/deps/tinyxml2")
    set (tinyxml2_LIBRARIES "")
    set (tinyxml2_LIBRARY_DIRS "")
  endif()

  if (NOT WIN32)
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
  else()
    set(libtar_LIBRARIES "")
  endif()

  #################################################
  # Find TBB
  pkg_check_modules(TBB tbb)
  set (TBB_PKG_CONFIG "tbb")
  if (NOT TBB_FOUND)
    message(STATUS "TBB not found, attempting to detect manually")
    set (TBB_PKG_CONFIG "")

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
  # On Windows, we assume that all the OGRE* defines are passed in manually
  # to CMake.
  if (NOT WIN32)
    execute_process(COMMAND pkg-config --modversion OGRE
                    OUTPUT_VARIABLE OGRE_VERSION)
    string(REPLACE "\n" "" OGRE_VERSION ${OGRE_VERSION})

    string (REGEX REPLACE "^([0-9]+).*" "\\1"
      OGRE_MAJOR_VERSION "${OGRE_VERSION}")
    string (REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
      OGRE_MINOR_VERSION "${OGRE_VERSION}")
    string (REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
      OGRE_PATCH_VERSION ${OGRE_VERSION})

    set(OGRE_VERSION
      ${OGRE_MAJOR_VERSION}.${OGRE_MINOR_VERSION}.${OGRE_PATCH_VERSION})
  endif()

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
  # On Windows, we assume that all the OGRE* defines are passed in manually
  # to CMake.
  if (NOT WIN32)
    execute_process(COMMAND pkg-config --variable=plugindir OGRE
                    OUTPUT_VARIABLE _pkgconfig_invoke_result
                    RESULT_VARIABLE _pkgconfig_failed)
    if(_pkgconfig_failed)
      BUILD_WARNING ("Failed to find OGRE's plugin directory.  The build will succeed, but gazebo will likely fail to run.")
    else()
      # This variable will be substituted into cmake/setup.sh.in
      set (OGRE_PLUGINDIR ${_pkgconfig_invoke_result})
    endif()
  endif()

  ########################################
  # Check and find libccd (if needed)
  pkg_check_modules(CCD ccd>=1.4)
  if (NOT CCD_FOUND)
    message(STATUS "Using internal copy of libccd")
    set(CCD_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/deps/libccd/include")
    set(CCD_LIBRARY_DIRS "${CMAKE_BINARY_DIR}/deps/libccd")
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
  # Find AV device. Only check for this on linux.
  if (UNIX)
    pkg_check_modules(libavdevice libavdevice>="56.4.100")
    if (NOT libavdevice_FOUND)
      BUILD_WARNING ("libavdevice not found. Recording to a video device will be disabled.")
    else()
      include_directories(${libavdevice_INCLUDE_DIRS})
      link_directories(${libavdevice_LIBRARY_DIRS})
    endif ()
  endif ()

  if (NOT libavdevice_FOUND)
    set (HAVE_AVDEVICE False)
  else()
    set (HAVE_AVDEVICE True)
  endif()

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

  if (libavutil_FOUND AND libavformat_FOUND AND libavcodec_FOUND AND
      libswscale_FOUND)
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

  if (BULLET_VERSION VERSION_GREATER 2.82)
    add_definitions( -DLIBBULLET_VERSION_GT_282 )
  endif()

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
else (PKG_CONFIG_FOUND)
  set (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
  BUILD_ERROR ("Error: pkg-config not found")
endif ()

########################################
# Find SDFormat
set (SDFormat_MIN_VERSION 4.1.0)
find_package(SDFormat ${SDFormat_MIN_VERSION})

if (NOT SDFormat_FOUND)
  message (STATUS "Looking for SDFormat - not found")
  BUILD_ERROR ("Missing: SDF version >=${SDFormat_MIN_VERSION}. Required for reading and writing SDF files.")
else()
  message (STATUS "Looking for SDFormat - found")
endif()

########################################
# Find QT
find_package(Qt4 COMPONENTS QtCore QtGui QtXml QtXmlPatterns REQUIRED)
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
  set(libdl_library "")
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
# Include man pages stuff
include (${gazebo_cmake_dir}/Ronn2Man.cmake)
include (${gazebo_cmake_dir}/Man.cmake)
add_manpage_target()

########################################
# Find Space Navigator header and library
find_library(SPNAV_LIBRARY NAMES spnav)
find_file(SPNAV_HEADER NAMES spnav.h)
if (SPNAV_LIBRARY AND SPNAV_HEADER)
  message(STATUS "Looking for libspnav and spnav.h - found")
  set(HAVE_SPNAV TRUE)
else()
  message(STATUS "Looking for libspnav and spnav.h - not found")
  set(HAVE_SPNAV FALSE)
endif()

########################################
# Find xsltproc, which is used by tools/check_test_ran.py
find_program(XSLTPROC xsltproc)
if (NOT EXISTS ${XSLTPROC})
  BUILD_WARNING("xsltproc not found. The check_test_ran.py script will cause tests to fail.")
endif()

########################################
# Find uuid-dev Library
#pkg_check_modules(uuid uuid)
#if (uuid_FOUND)
#  message (STATUS "Looking for uuid - found")
#  set (HAVE_UUID TRUE)
#else ()
#  set (HAVE_UUID FALSE)
#  BUILD_WARNING ("uuid-dev library not found - Gazebo will not have uuid support.")
#endif ()

########################################
# Find uuid
#  - In UNIX we use uuid library.
#  - In Windows the native RPC call, no dependency needed.
if (UNIX)
  pkg_check_modules(uuid uuid)
  if (uuid_FOUND)
    message (STATUS "Looking for uuid - found")
    set (HAVE_UUID TRUE)
  else ()
    set (HAVE_UUID FALSE)
    BUILD_WARNING ("uuid-dev library not found - Gazebo will not have uuid support.")
  endif ()
else()
  message (STATUS "Using Windows RPC UuidCreate function")
  set (HAVE_UUID TRUE)
endif()

########################################
# Find graphviz
include (${gazebo_cmake_dir}/FindGraphviz.cmake)
if (NOT GRAPHVIZ_FOUND)
  message (STATUS "Looking for libgraphviz-dev - not found")
  BUILD_WARNING ("Graphviz not found, Model editor's schematic view will be disabled.")
  set (HAVE_GRAPHVIZ OFF CACHE BOOL "HAVE GRAPHVIZ" FORCE)
else ()
  message (STATUS "Looking for libgraphviz-dev - found")
  set (HAVE_GRAPHVIZ ON CACHE BOOL "HAVE GRAPHVIZ" FORCE)
endif ()

########################################
# Find ignition msgs
find_package(ignition-msgs0 0.4 QUIET)
if (NOT ignition-msgs0_FOUND)
  message(STATUS "Looking for ignition-msgs0-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition msgs0 library.")
else()
  message(STATUS "Looking for ignition-msgs0-config.cmake - found")
endif()

########################################
# Find ignition math library
find_package(ignition-math2 2.6 QUIET)
if (NOT ignition-math2_FOUND)
  message(STATUS "Looking for ignition-math2-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition math2 library.")
else()
  message(STATUS "Looking for ignition-math2-config.cmake - found")
endif()

########################################
# Find the Ignition_Transport library
find_package(ignition-transport2 QUIET)
if (NOT ignition-transport2_FOUND)
  find_package(ignition-transport1 QUIET)
  if (NOT ignition-transport1_FOUND)
    BUILD_WARNING ("Missing: Ignition Transport (libignition-transport-dev or libignition-transport2-dev)")
  endif()
endif()

if (ignition-transport2_FOUND OR ignition-transport1_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
  include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
  link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
endif()

################################################
# Find Valgrind for checking memory leaks in the
# tests
find_program(VALGRIND_PROGRAM NAMES valgrind PATH ${VALGRIND_ROOT}/bin)
option(GAZEBO_RUN_VALGRIND_TESTS "Run gazebo tests with Valgrind" FALSE)
mark_as_advanced(GAZEBO_RUN_VALGRIND_TESTS)
if (GAZEBO_RUN_VALGRIND_TESTS AND NOT VALGRIND_PROGRAM)
  BUILD_WARNING("valgrind not found. Memory check tests will be skipped.")
endif()

########################################
# Find OSVR SDK
find_library(OSVR_CLIENTKIT_LIBRARY NAMES osvrClientKit)
find_file(OSVR_CLIENTKIT_HEADER NAMES osvr/ClientKit/ClientKit.h)
if (OSVR_CLIENTKIT_LIBRARY AND OSVR_CLIENTKIT_HEADER)
  message(STATUS "Looking for libosvrClientKit and ClientKit.h - found")
  set(HAVE_OSVR TRUE)
else()
  message(STATUS "Looking for libosvrClientKit and ClientKit.h - not found")
  BUILD_WARNING("OpenSource Virtual Reality (OSVR) support will be disabled.")
  set(HAVE_OSVR FALSE)
endif()

########################################
# Find QWT (QT graphing library)
find_path(QWT_INCLUDE_DIR NAMES qwt.h PATHS
  /usr/include
  /usr/local/include
  /usr/local/lib/qwt.framework/Headers
  ${QWT_WIN_INCLUDE_DIR}

  PATH_SUFFIXES qwt-qt4 qwt qwt5
)

find_library(QWT_LIBRARY NAMES qwt qwt6 qwt5 PATHS
  /usr/lib
  /usr/local/lib
  /usr/local/lib/qwt.framework
  ${QWT_WIN_LIBRARY_DIR}
)

if (QWT_INCLUDE_DIR AND QWT_LIBRARY)
  set(HAVE_QWT TRUE)
else()
  set(HAVE_QWT FALSE)
endif ()

# version
set ( _VERSION_FILE ${QWT_INCLUDE_DIR}/qwt_global.h )
file ( STRINGS ${_VERSION_FILE} _VERSION_LINE REGEX "define[ ]+QWT_VERSION_STR" )
if ( _VERSION_LINE )
  string ( REGEX REPLACE ".*define[ ]+QWT_VERSION_STR[ ]+\"(.*)\".*" "\\1"
      QWT_VERSION_STRING "${_VERSION_LINE}" )
  string ( REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+).*" "\\1"
      QWT_MAJOR_VERSION "${QWT_VERSION_STRING}" )
  string ( REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+).*" "\\2"
      QWT_MINOR_VERSION "${QWT_VERSION_STRING}" )
  string ( REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+).*" "\\3"
      QWT_PATCH_VERSION "${QWT_VERSION_STRING}" )
  set(QWT_VERSION
    ${QWT_MAJOR_VERSION}.${QWT_MINOR_VERSION}.${QWT_PATCH_VERSION})
endif ()

# in Windows, the path need to point to the parent to get correct qwt/foo headers
if (WIN32)
  SET(QWT_INCLUDE_DIR "${QWT_INCLUDE_DIR}\\..")	
endif()

if (HAVE_QWT)
  message (STATUS "Looking for qwt - found: version ${QWT_VERSION}")
else()
  message (STATUS "Looking for qwt - not found")
  BUILD_ERROR ("Missing: libqwt-dev. Required for plotting.")
endif ()
