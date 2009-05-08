INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE (FindFLTK)
INCLUDE (FindPkgConfig)

SET (INCLUDE_AV ON CACHE BOOL "Include audio/video functionality" FORCE)
SET (INCLUDE_WEBGAZEBO ON CACHE BOOL "Build webgazebo" FORCE)
SET (OGRE_LIBRARY_PATH "/usr/local/lib" CACHE INTERNAL "Ogre library path")

########################################
# Find packages
IF (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE>=${OGRE_VERSION})
  IF (NOT OGRE_FOUND)
    MESSAGE (SEND_ERROR "\nError: Ogre3d and development files not found. See the following website: http://www.orge3d.org")
  ELSE (NOT OGRE_FOUND)
 
    SET (OGRE_LIBRARY_PATH ${OGRE_LIBRARY_DIRS} CACHE INTERNAL "Ogre library path")
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
  ENDIF (NOT OGRE_FOUND)

  pkg_check_modules(ODE ode>=${ODE_VERSION})
  IF (NOT ODE_FOUND)
    MESSAGE (SEND_ERROR "\nError: ODE and development files not found. See the following website: http://www.ode.org")
  ELSE (NOT ODE_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${ODE_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${ODE_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${ODE_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${ODE_LIBRARIES})
  ENDIF (NOT ODE_FOUND)

  pkg_check_modules(XML libxml-2.0)
  IF (NOT XML_FOUND)
    MESSAGE (SEND_ERROR "\nError: libxml2 and development files not found. See the following website: http://www.xmlsoft.org")
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
    MESSAGE (SEND_ERROR "\nError: XFT and development files not found. See the following website: http://www.fontconfig.org")
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
    SET (INCLUDE_AV OFF CACHE BOOL "Include audio/video functionality" FORCE)
    MESSAGE (STATUS "Warning: Openal and development files not found. Audio capabilities will be disabled. See the following website: http://connect.creativelabs.com/openal/default.aspx")
  ELSE (NOT OAL_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_cflags 
                          ${gazeboserver_cflags_desc} "-DHAVE_OPENAL" )
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
    SET (INCLUDE_AV OFF CACHE BOOL "Include audio/video functionality" FORCE)
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
    SET (INCLUDE_AV OFF CACHE BOOL "Include audio/video functionality" FORCE)
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

  pkg_check_modules(PLAYER playerc++)
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
  MESSAGE (FATAL_ERROR "\nError: pkg-config not found")
  SET (BUILD_GAZEBO OFF CACHE BOOL "Build Gazebo" FORCE)
ENDIF (PKG_CONFIG_FOUND)


########################################
# Find Boost, if not specified manually
IF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )
  INCLUDE (FindBoost)
  FIND_PACKAGE( Boost 1.34.1 COMPONENTS thread signals)
  IF (NOT Boost_FOUND)
    MESSAGE (FATAL_ERROR "Boost thread and signals not found")
    SET (BUILD_GAZEBO OFF CACHE BOOL "Build Gazebo" FORCE)
  ENDIF (NOT Boost_FOUND)

  LIST_TO_STRING(tmp "${Boost_INCLUDE_DIRS}")
  SET (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  FOREACH (dir ${Boost_LIBRARY_DIRS})
    APPEND_TO_CACHED_STRING(boost_link_flags 
      "Boost link flags. Use this to override automatic detection." "-L${dir}" )
  ENDFOREACH (dir Boost_LIBRARY_DIRS)

  FOREACH (lib ${Boost_LIBRARIES})
    APPEND_TO_CACHED_STRING (boost_link_flags 
      "Boost link flags. Use this to override automatic detection." "-l${lib}" )
  ENDFOREACH (lib Boost_LIBRARIES)

ENDIF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )

MESSAGE (STATUS "Boost IDIRS ${boost_include_dirs}")
MESSAGE (STATUS "Boost LFLAGS ${boost_link_flags}")

########################################
# Find avformat and avcodec
IF (INCLUDE_AV)
  SET (libavformat_search_path 
    /usr/include /usr/include/libavformat /usr/local/include 
    /usr/local/include/libavformat
  )
  
  SET (libavcodec_search_path 
    /usr/include /usr/include/libavcodec /usr/local/include 
    /usr/local/include/libavcodec
  )
  
  FIND_PATH(LIBAVFORMAT_PATH avformat.h ${libavformat_search_path})
  IF (NOT LIBAVFORMAT_PATH)
    MESSAGE (STATUS "Looking for avformat.h - not found")
    MESSAGE (STATUS "  Warning: audio/video will not be built")
  ELSE (NOT LIBAVFORMAT_PATH)
    MESSAGE (STATUS "Looking for avformat.h - found")
  ENDIF (NOT LIBAVFORMAT_PATH)

  FIND_PATH(LIBAVCODEC_PATH avcodec.h ${libavcodec_search_path})
  IF (NOT LIBAVCODEC_PATH)
    MESSAGE (STATUS "Looking for avcodec.h - not found")
    MESSAGE (STATUS "  Warning: audio/video will not be built")
  ELSE (NOT LIBAVCODEC_PATH)
    MESSAGE (STATUS "Looking for avcodec.h - found")
  ENDIF (NOT LIBAVCODEC_PATH)

ENDIF (INCLUDE_AV)


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
# Find yaml
SET (libyaml_search_path /usr/include /usr/local/include)
FIND_PATH(LIBYAML_PATH yaml.h ${libyaml_search_path})
IF (NOT LIBYAML_PATH)
  MESSAGE (STATUS "Looking for yaml.h - not found")
  MESSAGE (STATUS "  Warning: webgazebo will not be built")
  SET (INCLUDE_WEBGAZEBO OFF CACHE BOOL "Found libevent" FORCE)
ELSE (NOT LIBYAML_PATH)
  MESSAGE (STATUS "Looking for yaml.h - found")
ENDIF (NOT LIBYAML_PATH)

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


