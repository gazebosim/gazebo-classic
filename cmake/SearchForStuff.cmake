INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE (FindFLTK)
INCLUDE (FindBoost)
INCLUDE (FindPkgConfig)

SET (INCLUDE_AV ON CACHE BOOL "Include audio/video functionality" FORCE)

########################################
# Find packages
IF (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE>=${OGRE_VERSION})
  IF (NOT OGRE_FOUND)
    MESSAGE (SEND_ERROR "\nError: Ogre3d and development files not found. See the following website: http://www.orge3d.org")
  ELSE (NOT OGRE_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${OGRE_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${OGRE_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${OGRE_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${OGRE_LIBRARIES})
  ENDIF (NOT OGRE_FOUND)

  pkg_check_modules(ODE ode>=${ODE_VERSION})
  IF (NOT ODE_FOUND)
    MESSAGE (SEND_ERROR "\nError: ODE and development files not found. See the following website: http://www.ode.org")
  ELSE (NOT ODE_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${ODE_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${ODE_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${ODE_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${ODE_LIBRARIES})
  ENDIF (NOT ODE_FOUND)

  pkg_check_modules(XML libxml-2.0)
  IF (NOT XML_FOUND)
    MESSAGE (SEND_ERROR "\nError: libxml2 and development files not found. See the following website: http://www.xmlsoft.org")
  ELSE (NOT XML_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${XML_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${XML_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${XML_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${XML_LIBRARIES})
  ENDIF (NOT XML_FOUND)

  pkg_check_modules(XFT xft)
  IF (NOT XFT_FOUND)
    MESSAGE (SEND_ERROR "\nError: XFT and development files not found. See the following website: http://www.fontconfig.org")
  ELSE (NOT XFT_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${XFT_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${XFT_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${XFT_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${XFT_LIBRARIES})
  ENDIF (NOT XFT_FOUND)

  pkg_check_modules(OAL openal)
  IF (NOT OAL_FOUND)
    SET (INCLUDE_AV OFF CACHE BOOL "Include audio/video functionality" FORCE)
    MESSAGE (STATUS "Warning: Openal and development files not found. Audio capabilities will be disabled. See the following website: http://connect.creativelabs.com/openal/default.aspx")
  ELSE (NOT OAL_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${OAL_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${OAL_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${OAL_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${OAL_LIBRARIES})
  ENDIF (NOT OAL_FOUND)

  pkg_check_modules(AVF libavformat)
  IF (NOT AVF_FOUND)
    SET (INCLUDE_AV OFF CACHE BOOL "Include audio/video functionality" FORCE)
    MESSAGE (STATUS "Warning: libavformat and development files not found. Audio capabilities will be disabled.")
  ELSE (NOT AVF_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${AVF_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${AVF_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${AVF_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${AVF_LIBRARIES})
  ENDIF (NOT AVF_FOUND)

  pkg_check_modules(AVC libavcodec)
  IF (NOT AVC_FOUND)
    SET (INCLUDE_AV OFF CACHE BOOL "Include audio/video functionality" FORCE)
    MESSAGE (STATUS "Warning: libavcodec and development files not found. Audio capabilities will be disabled.")
  ELSE (NOT AVC_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs "Include dirs" 
                          ${AVC_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs "Link dirs" 
                          ${AVC_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${AVC_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs "Link libs" ${AVC_LIBRARIES})
  ENDIF (NOT AVC_FOUND)

ELSE (PKG_CONFIG_FOUND)
  MESSAGE (FATAL_ERROR "\nError: pkg-config not found")
  SET (BUILD_GAZEBO OFF CACHE BOOL "Build Gazebo" FORCE)
ENDIF (PKG_CONFIG_FOUND)


########################################
# Find Boost
FIND_PACKAGE( Boost 1.34.1 COMPONENTS thread signals)
IF (NOT Boost_FOUND)
  MESSAGE (FATAL_ERROR "Boost thread and signals not found")
  SET (BUILD_GAZEBO OFF CACHE BOOL "Build Gazebo" FORCE)
ENDIF (NOT Boost_FOUND)

########################################
# Find avformat and avcodec
SET (libavformat_search_path 
  /usr/include /usr/include/libavformat /usr/local/include 
  /usr/local/include/libavformat
)

SET (libavcodec_search_path 
  /usr/include /usr/include/libavcodec /usr/local/include 
  /usr/local/include/libavcodec
)

FIND_PATH(LIBAVFORMAT_PATH avformat.h ${libavformat_search_path})
FIND_PATH(LIBAVCODEC_PATH avcodec.h ${libavcodec_search_path})
