INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE (${gazebo_cmake_dir}/FindOS.cmake)
SET(FLTK_SKIP_FLUID TRUE)
INCLUDE (FindFLTK)
INCLUDE (FindPkgConfig)
INCLUDE (${gazebo_cmake_dir}/FindOde.cmake)
INCLUDE (${gazebo_cmake_dir}/FindFreeimage.cmake)

#if (NOT FLTK_FOUND)
#  BUILD_ERROR("FLTK libraries and development files not found. See the following website for installation instructions: http://fltk.org")
#endif (NOT FLTK_FOUND)


set (use_internal_assimp OFF CACHE BOOL "Use internal assimp" FORCE)

set (INCLUDE_WEBGAZEBO ON CACHE BOOL "Build webgazebo" FORCE)
set (OGRE_LIBRARY_PATH "/usr/local/lib" CACHE INTERNAL "Ogre library path")

set (assimp_include_dirs "" CACHE STRING "Assimp include paths. Use this to override automatic detection." FORCE)
set (assimp_library_dirs "" CACHE STRING "Assimp library paths. Use this to override automatic detection." FORCE)
set (assimp_libraries "" CACHE STRING "Assimp libraries Use this to override automatic detection." FORCE)

set (boost_include_dirs "" CACHE STRING "Boost include paths. Use this to override automatic detection.")
set (boost_library_dirs "" CACHE STRING "Boost library paths. Use this to override automatic detection.")
set (boost_libraries "" CACHE STRING "Boost libraries. Use this to override automatic detection.")
set (bullet_dynamics_dirs "" CACHE STRING "Bullet Dynamics libraries. Use this to override automatic detection.")
set (bullet_collision_dirs "" CACHE STRING "Bullet Collision libraries. Use this to override automatic detection.")
set (bullet_softbody_dirs "" CACHE STRING "Bullet Softbody libraries. Use this to override automatic detection.")
set (bullet_math_dirs "" CACHE STRING "Bullet LinearMath libraries. Use this to override automatic detection.")
set (bullet_cflags "-DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX" CACHE STRING "Bullet Dynamics C compile flags exported by rospack.")
set (threadpool_include_dirs "" CACHE STRING "Threadpool include paths. Use this to override automatic detection.")

########################################
# Find packages
if (PKG_CONFIG_FOUND)

  pkg_check_modules(OGRE OGRE>=${MIN_OGRE_VERSION})
  if (NOT OGRE_FOUND)
    BUILD_ERROR("Ogre3d version >=${MIN_OGRE_VERSION} and development files not found. See the following website for installation instructions: http://www.orge3d.org")
  else (NOT OGRE_FOUND)
 
    set (OGRE_LIBRARY_PATH ${OGRE_LIBRARY_DIRS} CACHE INTERNAL "Ogre library path")

    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${OGRE_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${OGRE_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${OGRE_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${OGRE_LIBRARIES})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${OGRE_LDFLAGS})
  endif (NOT OGRE_FOUND)

  pkg_check_modules(XML libxml-2.0)
  IF (NOT XML_FOUND)
    BUILD_ERROR("libxml2 and development files not found. See the following website: http://www.xmlsoft.org")
  ELSE (NOT XML_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${XML_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${XML_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${XML_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${XML_LIBRARIES})
  ENDIF (NOT XML_FOUND)

  pkg_check_modules(XFT xft)
  IF (NOT XFT_FOUND)
    BUILD_ERROR("XFT and development files not found. See the following website: http://www.fontconfig.org")
  ELSE (NOT XFT_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${XFT_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${XFT_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${XFT_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${XFT_LIBRARIES})
  ENDIF (NOT XFT_FOUND)

  pkg_check_modules(OAL openal)
  IF (NOT OAL_FOUND)
    MESSAGE (STATUS "Warning: Openal and development files not found. Audio capabilities will be disabled. See the following website: http://connect.creativelabs.com/openal/default.aspx")
  ELSE (NOT OAL_FOUND)
    SET (HAVE_OPENAL TRUE)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${OAL_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${OAL_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${OAL_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${OAL_LIBRARIES})
  ENDIF (NOT OAL_FOUND)

  pkg_check_modules(AVF libavformat)
  IF (NOT AVF_FOUND)
    MESSAGE (STATUS "Warning: libavformat and development files not found. Audio capabilities will be disabled.")
  ELSE (NOT AVF_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${AVF_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${AVF_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${AVF_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${AVF_LIBRARIES})
  ENDIF (NOT AVF_FOUND)

  pkg_check_modules(AVC libavcodec)
  IF (NOT AVC_FOUND)
    MESSAGE (STATUS "Warning: libavcodec and development files not found. Audio capabilities will be disabled.")
  ELSE (NOT AVC_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${AVC_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${AVC_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${AVC_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${AVC_LIBRARIES})
  ENDIF (NOT AVC_FOUND)

  IF (AVF_FOUND AND AVC_FOUND)
    SET (HAVE_FFMPEG TRUE)
  ENDIF (AVF_FOUND AND AVC_FOUND)

  pkg_check_modules(PLAYER playercore>=3.0)
  IF (NOT PLAYER_FOUND)
    SET (INCLUDE_PLAYER OFF CACHE BOOL "Build gazebo plugin for player" FORCE)
    MESSAGE (STATUS "Warning: Player not found. The gazebo plugin for player will not be built. See the following website: http://playerstage.sourceforge.net")
  ELSE (NOT PLAYER_FOUND)
    SET (INCLUDE_PLAYER ON CACHE BOOL "Build gazebo plugin for player" FORCE)
    SET (PLAYER_INCLUDE_DIRS ${PLAYER_INCLUDE_DIRS} CACHE INTERNAL
         "Player include directory")
    SET (PLAYER_LINK_DIRS ${PLAYER_LINK_DIRS} CACHE INTERNAL
         "Player link directory")
    SET (PLAYER_LINK_LIBS ${PLAYER_LIBRARIES} CACHE INTERNAL
         "Player libraries")
  ENDIF (NOT PLAYER_FOUND)

  pkg_check_modules(WEBSIM websim)
  IF (NOT WEBSIM_FOUND)
    SET (INCLUDE_WEBGAZEBO OFF CACHE BOOL "Build webgazebo" FORCE)
    MESSAGE (STATUS "Warning: Websim not found. Webgazebo will not be built")
  ELSE (NOT WEBSIM_FOUND)
    SET (WEBSIM_INCLUDE_DIRS ${WEBSIM_INCLUDE_DIRS} CACHE INTERNAL
         "Websim include directory")
    SET (WEBSIM_LINK_DIRS ${WEBSIM_LINK_DIRS} CACHE INTERNAL 
         "Websim link directory")
    SET (WEBSIM_LINK_LIBS ${WEBSIM_LIBRARIES} CACHE INTERNAL
         "Websim libraries")
  ENDIF (NOT WEBSIM_FOUND)

ELSE (PKG_CONFIG_FOUND)
  SET (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
  MESSAGE (FATAL_ERROR "\nError: pkg-config not found")
ENDIF (PKG_CONFIG_FOUND)


########################################
# Find Boost, if not specified manually
IF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )

  # Clear some variables to ensure that the checks for boost are 
  # always run
  SET (Boost_THREAD_FOUND OFF CACHE INTERNAL "" FORCE)
  SET (Boost_SIGNALS_FOUND OFF CACHE INTERNAL "" FORCE)

  SET(Boost_ADDITIONAL_VERSIONS "1.35" "1.35.0" "1.36" "1.36.1" "1.37.0" "1.39.0" CACHE INTERNAL "Boost Additional versions" FORCE)
  INCLUDE (FindBoost)

  FIND_PACKAGE( Boost ${MIN_BOOST_VERSION} REQUIRED thread signals )

  IF (NOT Boost_FOUND)
    SET (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
    MESSAGE (FATAL_ERROR "Boost thread and signals not found. Please install Boost threads and signals version ${MIN_BOOST_VERSION} or higher.")
  ENDIF (NOT Boost_FOUND)

  SET (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  SET (boost_library_dirs ${Boost_LIBRARY_DIRS} CACHE STRING
    "Boost link dirs. Use this to override automatic detection." FORCE)

  LIST_TO_STRING(tmp "${Boost_LIBRARIES}")
  SET (boost_libraries ${tmp} CACHE STRING 
    "Boost libraries. Use this to override automatic detection." FORCE )

ENDIF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries ) 

STRING(REGEX REPLACE "(^| )-L" " " boost_library_dirs "${boost_library_dirs}")
STRING(REGEX REPLACE "(^| )-l" " " boost_libraries "${boost_libraries}")
#STRING(STRIP ${boost_library_dirs} boost_library_dirs)
#STRING(STRIP ${boost_libraries} boost_libraries)
STRING(REGEX REPLACE " " ";" boost_libraries "${boost_libraries}")

MESSAGE (STATUS "Boost Include Path: ${boost_include_dirs}")
MESSAGE (STATUS "Boost Library Path: ${boost_library_dirs}")
MESSAGE (STATUS "Boost Libraries: ${boost_libraries}")

########################################
# For Threadpool
message (STATUS "Threadpool Include Path: ${threadpool_include_dirs}")

########################################
# Find avformat and avcodec
IF (HAVE_FFMPEG)
  SET (libavformat_search_path 
    /usr/include /usr/include/libavformat /usr/local/include 
    /usr/local/include/libavformat /usr/include/ffmpeg
  )
  
  SET (libavcodec_search_path 
    /usr/include /usr/include/libavcodec /usr/local/include 
    /usr/local/include/libavcodec /usr/include/ffmpeg
  )
  
  FIND_PATH(LIBAVFORMAT_PATH avformat.h ${libavformat_search_path})
  IF (NOT LIBAVFORMAT_PATH)
    MESSAGE (STATUS "Looking for avformat.h - not found")
    MESSAGE (STATUS "  Warning: audio/video will not be built")
    SET (LIBAVFORMAT_PATH /usr/include)
  ELSE (NOT LIBAVFORMAT_PATH)
    MESSAGE (STATUS "Looking for avformat.h - found")
  ENDIF (NOT LIBAVFORMAT_PATH)

  FIND_PATH(LIBAVCODEC_PATH avcodec.h ${libavcodec_search_path})
  IF (NOT LIBAVCODEC_PATH)
    MESSAGE (STATUS "Looking for avcodec.h - not found")
    MESSAGE (STATUS "  Warning: audio/video will not be built")
    SET (LIBAVCODEC_PATH /usr/include)
  ELSE (NOT LIBAVCODEC_PATH)
    MESSAGE (STATUS "Looking for avcodec.h - found")
  ENDIF (NOT LIBAVCODEC_PATH)

ELSE (HAVE_FFMPEG)
  SET (LIBAVFORMAT_PATH /usr/include)
  SET (LIBAVCODEC_PATH /usr/include)
ENDIF (HAVE_FFMPEG)


########################################
# Find libevent
SET (libevent_search_path /usr/include /usr/local/include)
FIND_PATH(LIBEVENT_PATH event.h ${libevent_search_path})
IF (NOT LIBEVENT_PATH)
  MESSAGE (STATUS "Looking for event.h - not found")
  MESSAGE (STATUS "  Warning: webgazebo will not be built")
  SET (INCLUDE_WEBGAZEBO OFF CACHE BOOL "Found libevent" FORCE)
ELSE (NOT LIBEVENT_PATH)
  MESSAGE (STATUS "Looking for event.h - found")
ENDIF (NOT LIBEVENT_PATH)

########################################
# Find profiler library, optional
FIND_LIBRARY(PROFILER "profiler")
IF (PROFILER)
  SET (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -lprofiler" 
       CACHE INTERNAL "Link flags for profile" FORCE)
ENDIF (PROFILER)

########################################
# Find tcmalloc library, optional
FIND_LIBRARY(TCMALLOC "tcmalloc")
IF (TCMALLOC)
  SET (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -ltcmalloc" 
       CACHE INTERNAL "Link flags for profile" FORCE)
ENDIF (TCMALLOC)

########################################
# Find libtool
FIND_PATH(libtool_include_dir ltdl.h /usr/include /usr/local/include)
IF (NOT libtool_include_dir)
  MESSAGE (STATUS "Looking for ltdl.h - not found")
  MESSAGE (STATUS "Warning: Unable to find libtool, plugins will not be supported.")
  SET (libtool_include_dir /usr/include)
ELSE (NOT libtool_include_dir)
  MESSAGE (STATUS "Looking for ltdl.h - found")
ENDIF (NOT libtool_include_dir)

FIND_LIBRARY(libtool_library ltdl /usr/lib /usr/local/lib)
IF (NOT libtool_library)
  MESSAGE (STATUS "Looking for libltdl - not found")
  MESSAGE (STATUS "Warning: Unable to find libtool, plugins will not be supported.")
ELSE (NOT libtool_library)
  MESSAGE (STATUS "Looking for libltdl - found")
ENDIF (NOT libtool_library)

IF (libtool_library AND libtool_include_dir)
  SET (HAVE_LTDL TRUE)
ENDIF (libtool_library AND libtool_include_dir)

########################################
# Find libdl
FIND_PATH(libdl_include_dir dlfcn.h /usr/include /usr/local/include)
IF (NOT libdl_include_dir)
  MESSAGE (STATUS "Looking for dlfcn.h - not found")
  MESSAGE (STATUS "Warning: Unable to find libdl, plugins will not be supported.")
  SET (libdl_include_dir /usr/include)
ELSE (NOT libdl_include_dir)
  MESSAGE (STATUS "Looking for dlfcn.h - found")
ENDIF (NOT libdl_include_dir)

FIND_LIBRARY(libdl_library dl /usr/lib /usr/local/lib)
IF (NOT libdl_library)
  MESSAGE (STATUS "Looking for libdl - not found")
  MESSAGE (STATUS "Warning: Unable to find libdl, plugins will not be supported.")
ELSE (NOT libdl_library)
  MESSAGE (STATUS "Looking for libdl - found")
ENDIF (NOT libdl_library)

IF (libdl_library AND libdl_include_dir)
  SET (HAVE_DL TRUE)
ENDIF (libdl_library AND libdl_include_dir)

########################################
# Find assimp
if (NOT assimp_include_dirs AND NOT assimp_library_dirs AND NOT assimp_libraries )

  find_path(assimp_include_dir assimp.hpp ${assimp_include_dirs} ENV CPATH)
  
  if (NOT assimp_include_dir)
    #BUILD_ERROR("assimp not found. See the following website for installation instructions: http://assimp.sourceforge.net")
    message (STATUS "Looking for assimp.hpp - not found. Using built in version.")
    set (assimp_include_dirs /usr/include CACHE STRING
      "Assimp include paths. Use this to override automatic detection.")
  else (NOT assimp_include_dir)
    message (STATUS "Looking for assimp.hpp - found")
    set (assim_include_dirs ${assimp_include_dir} CACHE STRING
      "Assimp include paths. Use this to override automatic detection.")
  endif (NOT assimp_include_dir)
  
  find_library(assimp_library assimp ENV LD_LIBRARY_PATH)
  
  if (NOT assimp_library)
    message (STATUS "Looking for libassimp - not found. Using builtin version.")
    #BUILD_ERROR("libassimp not found. See the following website for installation instructions: http://assimp.sourceforge.net")
  else (NOT assimp_library)
    message (STATUS "Looking for libassimp - found")
    APPEND_TO_CACHED_LIST(assimp_libraries
                          "Assimp libraries Use this to override automatic detection."
                          ${assimp_library})
  endif (NOT assimp_library)
 
  if (NOT assimp_include_dir OR NOT assimp_library)
    set (use_internal_assimp ON CACHE BOOL "Use internal assimp" FORCE)
  endif (NOT assimp_include_dir OR NOT assimp_library)

endif (NOT assimp_include_dirs AND NOT assimp_library_dirs AND NOT assimp_libraries )

########################################
# Find bullet
FIND_PATH( bullet_include_dir btBulletDynamicsCommon.h ${bullet_include_dirs} ENV CPATH)
IF (NOT bullet_include_dir)
  MESSAGE (STATUS "Looking for btBulletDynamicsCommon.h - not found")
  SET (bullet_include_dir /usr/include)
ELSE (NOT bullet_include_dir)
  MESSAGE (STATUS "Looking for btBulletDynamicsCommon.h - found")
ENDIF (NOT bullet_include_dir)

FIND_LIBRARY(bullet_dynamics_library NAMES BulletDynamics 
  PATHS ${bullet_dynamics_dirs} ENV LD_LIBRARY_PATH)
FIND_LIBRARY(bullet_collision_library NAMES BulletCollision  
  PATHS ${bullet_collision_dirs} ENV LD_LIBRARY_PATH)
FIND_LIBRARY(bullet_softbody_library NAMES BulletSoftBody  
  PATHS ${bullet_softbody_dirs} ENV LD_LIBRARY_PATH)
FIND_LIBRARY(bullet_math_library NAMES LinearMath
  PATHS ${bullet_math_dirs} ENV LD_LIBRARY_PATH)

IF (NOT bullet_dynamics_library OR 
    NOT bullet_collision_library OR 
    NOT bullet_softbody_library OR
    NOT bullet_math_library)
  MESSAGE (STATUS "Looking for bullet libraries - not found")
ELSE (NOT bullet_dynamics_library OR 
      NOT bullet_collision_library OR 
      NOT bullet_softbody_library OR
      NOT bullet_math_library)
  MESSAGE (STATUS "Looking for bullet libraries - found")
ENDIF (NOT bullet_dynamics_library OR 
       NOT bullet_collision_library OR 
       NOT bullet_softbody_library OR
       NOT bullet_math_library)

IF (bullet_dynamics_library AND 
    bullet_collision_library AND 
    bullet_softbody_library  AND 
    bullet_math_library AND
    bullet_include_dir)
  SET (INCLUDE_BULLET TRUE CACHE BOOL "Include support for Bullet")

  APPEND_TO_CACHED_LIST(bullet_link_libs 
                        ${bullet_link_libs_desc} 
                        ${bullet_dynamics_library})
  APPEND_TO_CACHED_LIST(bullet_link_libs 
                        ${bullet_link_libs_desc} 
                        ${bullet_collision_library})
  APPEND_TO_CACHED_LIST(bullet_link_libs 
                        ${bullet_link_libs_desc} 
                        ${bullet_softbody_library})
  APPEND_TO_CACHED_LIST(bullet_link_libs 
                        ${bullet_link_libs_desc} 
                        ${bullet_math_library})


ELSE (bullet_dynamics_library AND 
    bullet_collision_library AND 
    bullet_softbody_library  AND 
    bullet_math_library AND
    bullet_include_dir)
  SET (INCLUDE_BULLET FALSE CACHE BOOL "Include support for Bullet")
  MESSAGE (STATUS "Warning: Unable to find bullet. The bullet physics engine will not be supported.")
ENDIF (bullet_dynamics_library AND 
       bullet_collision_library AND 
       bullet_softbody_library  AND 
       bullet_math_library AND
       bullet_include_dir)
