include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${gazebo_cmake_dir}/FindSSE.cmake)
include (CheckCXXSourceCompiles)

set (OGRE_LIBRARY_PATH "/usr/local/lib" CACHE INTERNAL "Ogre library path")

set (tinyxml_include_dirs "" CACHE STRING "Tinyxml include paths. Use this to override automatic detection.")
set (tinyxml_library_dirs "" CACHE STRING "Tinyxml library paths. Use this to override automatic detection.")
set (tinyxml_libraries "" CACHE STRING "Tinyxml libraries Use this to override automatic detection.")
set (tinyxml_cflags "" CACHE STRING "Tinyxml Use this cflag to enable string support.")

set (boost_include_dirs "" CACHE STRING "Boost include paths. Use this to override automatic detection.")
set (boost_library_dirs "" CACHE STRING "Boost library paths. Use this to override automatic detection.")
set (boost_libraries "" CACHE STRING "Boost libraries. Use this to override automatic detection.")

SET (gazebo_lflags "" CACHE STRING "Linker flags such as rpath for gazebo executable.")

SET (general_libraries "" CACHE STRING "general libraries")

include (${gazebo_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)
include (${gazebo_cmake_dir}/FindFreeimage.cmake)

execute_process(COMMAND pkg-config --modversion protobuf 
  OUTPUT_VARIABLE PROTOBUF_VERSION
  RESULT_VARIABLE protobuf_modversion_failed)

if (PROTOBUF_VERSION LESS 2.3.0)
  BUILD_ERROR("Incorrect version: Gazebo requires protobuf version 2.3.0 or greater")
endif()

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
# Find packages
if (PKG_CONFIG_FOUND)

  pkg_check_modules(PROFILER libprofiler)
  if (PROFILER_FOUND)
    # APPEND_TO_CACHED_LIST(general_libraries "general libraries" profiler)
    set (CMAKE_LINK_FLAGS_PROFILE "-Wl,--no-as-needed -lprofiler -Wl,--as-needed ${CMAKE_LINK_FLAGS_PROFILE}" CACHE INTERNAL "Link flags for profile")
  else ()
    find_library(PROFILER profiler)
    if (PROFILER)
      message (STATUS "Looking for libprofiler - found")
      # APPEND_TO_CACHED_LIST(general_libraries "general libraries" profiler)
      set (CMAKE_LINK_FLAGS_PROFILE "-Wl,--no-as-needed -lprofiler -Wl,--as-needed ${CMAKE_LINK_FLAGS_PROFILE}" CACHE INTERNAL "Link flags for profile")
    else()
      message (STATUS "Looking for libprofiler - not found")
    endif()
  endif()

  pkg_check_modules(TCMALLOC libtcmalloc)
  if (TCMALLOC_FOUND)
    # APPEND_TO_CACHED_LIST(general_libraries "general libraries" tcmalloc)
    set (CMAKE_LINK_FLAGS_PROFILE "${CMAKE_LINK_FLAGS_PROFILE} -Wl,--no-as-needed -ltcmalloc -Wl,--no-as-needed"
      CACHE INTERNAL "Link flags for profile" FORCE)
  else ()
    find_library(TCMALLOC tcmalloc)
    if (TCMALLOC)
      message (STATUS "Looking for libtcmalloc - found")
      # APPEND_TO_CACHED_LIST(general_libraries "general libraries" tcmalloc)
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
    set (HAVE_CEGUI FALSE)
  else()
    message (STATUS "Looking for CEGUI, found")
    if (NOT CEGUI_OGRE_FOUND)
      BUILD_WARNING ("CEGUI-OGRE not found, opengl GUI will be disabled.")
      set (HAVE_CEGUI FALSE)
    else()
      set (HAVE_CEGUI TRUE)
      set (CEGUI_LIBRARIES "CEGUIBase;CEGUIOgreRenderer")
      message (STATUS "Looking for CEGUI-OGRE, found")
    endif()
  endif()

  #################################################
  # Find bullet
  pkg_check_modules(BULLET bullet)
  if (BULLET_FOUND)
    set (HAVE_BULLET TRUE)
  else()
    set (HAVE_BULLET FALSE)
  endif()
  
  #################################################
  # Find tinyxml
  pkg_check_modules(tinyxml tinyxml)
  if (NOT tinyxml_FOUND)
    message(STATUS "tinyxml system package not found, using passed in paths")
    ########################################
    # Find tinyxml
    if (NOT tinyxml_include_dirs AND NOT tinyxml_library_dirs AND NOT tinyxml_libraries )

      message(STATUS "tinyxml has no passed in paths, try to auto detect manually.")

      find_path(tinyxml_include_dir tinyxml/tinyxml.hpp ${tinyxml_include_dirs} ENV CPATH)
      
      if (NOT tinyxml_include_dir)
        message (STATUS "Looking for tinyxml/tinyxml.hpp - not found.")
        set (tinyxml_include_dirs /usr/include CACHE STRING
          "tinyxml include paths. Use this to override automatic detection.")
      else (NOT tinyxml_include_dir)
        message (STATUS "Looking for tinyxml/tinyxml.hpp - found")
        set (assim_include_dirs ${tinyxml_include_dir} CACHE STRING
          "tinyxml include paths. Use this to override automatic detection.")
      endif (NOT tinyxml_include_dir)
      
      find_library(tinyxml_library tinyxml ENV LD_LIBRARY_PATH)
      
      if (tinyxml_library)
        message (STATUS "Looking for libtinyxml - found")
        APPEND_TO_CACHED_LIST(tinyxml_libraries
                              "tinyxml libraries Use this to override automatic detection."
                              ${tinyxml_library})
      endif (tinyxml_library)
     
      if (NOT tinyxml_include_dir OR NOT tinyxml_library)
        BUILD_ERROR("Missing: tinyxml")
      endif (NOT tinyxml_include_dir OR NOT tinyxml_library)

    endif (NOT tinyxml_include_dirs AND NOT tinyxml_library_dirs AND NOT tinyxml_libraries )
  else ()
    set(tinyxml_include_dirs ${tinyxml_INCLUDE_DIRS} CACHE STRING "Tinyxml include paths. Use this to override automatic detection." FORCE)

    set(tinyxml_library_dirs ${tinyxml_LIBRARY_DIRS} CACHE STRING "Tinyxml library paths. Use this to override automatic detection." FORCE)

    set(tinyxml_libraries ${tinyxml_LIBRARIES} CACHE STRING "Tinyxml libraries Use this to override automatic detection." FORCE)

    set(tinyxml_cflags ${tinyxml_CFLAGS} CACHE STRING "Tinyxml Use this cflag to enable string support." FORCE)
  endif ()


  #################################################
  # Find TBB
  pkg_check_modules(TBB tbb)
  IF (NOT TBB_FOUND)
    BUILD_ERROR ("Missing: TBB - Threading Building Blocks")
  ENDIF (NOT TBB_FOUND)

  #################################################
  # Find OGRE 
  pkg_check_modules(OGRE-RTShaderSystem OGRE-RTShaderSystem>=${MIN_OGRE_VERSION})
  if (OGRE-RTShaderSystem_FOUND)
    set(ogre_ldflags ${OGRE-RTShaderSystem_LDFLAGS})
    set(ogre_include_dirs ${OGRE-RTShaderSystem_INCLUDE_DIRS})
    set(ogre_library_dirs ${OGRE-RTShaderSystem_LIBRARY_DIRS})
    set(ogre_libraries "OgreRTShaderSystem")
    set(ogre_cflags ${OGRE-RTShaderSystem_CFLAGS})

    set (INCLUDE_RTSHADER ON CACHE BOOL "Enable GPU shaders")

  else ()

    set (INCLUDE_RTSHADER OFF CACHE BOOL "Enable GPU shaders")

    pkg_check_modules(OGRE OGRE>=${MIN_OGRE_VERSION})
    if (NOT OGRE_FOUND)
      BUILD_ERROR("Missing: Ogre3d version >=${MIN_OGRE_VERSION}(http://www.orge3d.org)")
    else (NOT OGRE_FOUND)
      set(ogre_ldflags ${OGRE_LDFLAGS})
      set(ogre_include_dirs ${OGRE_INCLUDE_DIRS})
      set(ogre_library_dirs ${OGRE_LIBRARY_DIRS})
      set(ogre_cflags ${OGRE_CFLAGS})
    endif (NOT OGRE_FOUND)
  endif (OGRE-RTShaderSystem_FOUND)

  # Bug in libogre-dev ubuntu install package which cause ogre_library_dirs to 
  # be empty
  if ("${ogre_library_dirs}" STREQUAL "")
    set (ogre_library_dirs "/usr/lib")
  endif()

  set (OGRE_LIBRARY_PATH ${ogre_library_dirs} CACHE INTERNAL "Ogre library path")
  set (OGRE_INCLUDE_DIRS ${ogre_include_dirs} CACHE INTERNAL "Ogre include path")

  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  if (OGRE-Terrain_FOUND)
    set(ogre_ldflags ${ogre_ldflags} ${OGRE-Terrain_LDFLAGS})
    set(ogre_include_dirs ${ogre_include_dirs} ${OGRE-Terrain_INCLUDE_DIRS})
    set(ogre_library_dirs ${ogre_library_dirs} ${OGRE-Terrain_LIBRARY_DIRS})
    set(ogre_libraries "${ogre_libraries};OgreTerrain;OgrePaging")
    set(ogre_cflags ${ogre_cflags} ${OGRE-Terrain_CFLAGS})
  endif()

  set(ogre_libraries "${ogre_libraries};OgreMain")

  #################################################
  # Find XML
  pkg_check_modules(XML libxml-2.0)
  if (NOT XML_FOUND)
    BUILD_ERROR("Missing: libxml2(http://www.xmlsoft.org)")
  endif (NOT XML_FOUND)


  ########################################
  # Find OpenAL
  # pkg_check_modules(OAL openal)
  # if (NOT OAL_FOUND)
  #   BUILD_WARNING ("Openal not found. Audio capabilities will be disabled.")
  #   set (HAVE_OPENAL FALSE)
  # else (NOT OAL_FOUND)
  #   set (HAVE_OPENAL TRUE)
  # endif (NOT OAL_FOUND)

  ########################################
  # Find AV format
  pkg_check_modules(AVF libavformat)
  if (NOT AVF_FOUND)
    BUILD_WARNING ("libavformat not found. Audio capabilities will be disabled.")
  endif (NOT AVF_FOUND)

  ########################################
  # Find avcodec
  pkg_check_modules(AVC libavcodec)
  if (NOT AVC_FOUND)
    BUILD_WARNING ("libavcodec not found. Audio capabilities will be disabled.")
  endif (NOT AVC_FOUND)

  if (AVF_FOUND AND AVC_FOUND)
    set (HAVE_FFMPEG TRUE)
  endif (AVF_FOUND AND AVC_FOUND)

  ########################################
  # Find Player
  pkg_check_modules(PLAYER playercore>=3.0 playerc++)
  if (NOT PLAYER_FOUND)
    set (INCLUDE_PLAYER OFF CACHE BOOL "Build gazebo plugin for player" FORCE)
    BUILD_WARNING ("Player not found, gazebo plugin for player will not be built.")
  else (NOT PLAYER_FOUND)
    set (INCLUDE_PLAYER ON CACHE BOOL "Build gazebo plugin for player" FORCE)
    set (PLAYER_INCLUDE_DIRS ${PLAYER_INCLUDE_DIRS} CACHE INTERNAL
         "Player include directory")
    set (PLAYER_LINK_DIRS ${PLAYER_LINK_DIRS} CACHE INTERNAL
         "Player link directory")
    set (PLAYER_LINK_LIBS ${PLAYER_LIBRARIES} CACHE INTERNAL
         "Player libraries")
  endif (NOT PLAYER_FOUND)

else (PKG_CONFIG_FOUND)
  set (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
  BUILD_ERROR ("Error: pkg-config not found")
endif (PKG_CONFIG_FOUND)

find_package (Qt4)
if (NOT QT4_FOUND)
  BUILD_ERROR("Missing: Qt4")
endif()

########################################
# Find Boost, if not specified manually
if (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )

  # Clear some variables to ensure that the checks for boost are 
  # always run
  set (Boost_THREAD_FOUND OFF CACHE INTERNAL "" FORCE)
  set (Boost_SIGNALS_FOUND OFF CACHE INTERNAL "" FORCE)

  set(Boost_ADDITIONAL_VERSIONS "1.35" "1.35.0" "1.36" "1.36.1" "1.37.0" "1.39.0" CACHE INTERNAL "Boost Additional versions" FORCE)

  include (FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED thread signals system filesystem program_options regex)

  if (NOT Boost_FOUND)
    set (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
    BUILD_ERROR ("Boost thread and signals not found. Please install Boost threads and signals version ${MIN_BOOST_VERSION} or higher.")
  endif (NOT Boost_FOUND)

  set (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  set (boost_library_dirs ${Boost_LIBRARY_DIRS} CACHE STRING
    "Boost link dirs. Use this to override automatic detection." FORCE)

  LIST_TO_STRING(tmp "${Boost_LIBRARIES}")
  set (boost_libraries ${tmp} CACHE STRING 
    "Boost libraries. Use this to override automatic detection." FORCE )

endif (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries ) 

set (Boost_DIR "" CACHE INTERNAL "" FORCE)

STRING(REGEX REPLACE "(^| )-L" " " boost_library_dirs "${boost_library_dirs}")
STRING(REGEX REPLACE "(^| )-l" " " boost_libraries "${boost_libraries}")
#STRING(STRIP ${boost_library_dirs} boost_library_dirs)
#STRING(STRIP ${boost_libraries} boost_libraries)
STRING(REGEX REPLACE " " ";" boost_libraries "${boost_libraries}")

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
    BUILD_WARNING ("avformat.h not found. audio/video will not be built")
    SET (LIBAVFORMAT_PATH /usr/include)
  ELSE (NOT LIBAVFORMAT_PATH)
    MESSAGE (STATUS "Looking for avformat.h - found")
  ENDIF (NOT LIBAVFORMAT_PATH)

  FIND_PATH(LIBAVCODEC_PATH avcodec.h ${libavcodec_search_path})
  IF (NOT LIBAVCODEC_PATH)
    MESSAGE (STATUS "Looking for avcodec.h - not found")
    BUILD_WARNING ("avcodec.h not found. audio/video will not be built")
    SET (LIBAVCODEC_PATH /usr/include)
  ELSE (NOT LIBAVCODEC_PATH)
    MESSAGE (STATUS "Looking for avcodec.h - found")
  ENDIF (NOT LIBAVCODEC_PATH)

ELSE (HAVE_FFMPEG)
  SET (LIBAVFORMAT_PATH /usr/include)
  SET (LIBAVCODEC_PATH /usr/include)
ENDIF (HAVE_FFMPEG)

########################################
# Find libtool
find_path(libtool_include_dir ltdl.h /usr/include /usr/local/include)
if (NOT libtool_include_dir)
  message (STATUS "Looking for ltdl.h - not found")
  BUILD_WARNING ("ltdl.h not found")
  set (libtool_include_dir /usr/include)
else (NOT libtool_include_dir)
  message (STATUS "Looking for ltdl.h - found")
endif (NOT libtool_include_dir)

find_library(libtool_library ltdl /usr/lib /usr/local/lib)
if (NOT libtool_library)
  message (STATUS "Looking for libltdl - not found")
else (NOT libtool_library)
  message (STATUS "Looking for libltdl - found")
endif (NOT libtool_library)

if (libtool_library AND libtool_include_dir)
  set (HAVE_LTDL TRUE)
else ()
  set (HAVE_LTDL FALSE)
  set (libtool_library "" CACHE STRING "" FORCE)
endif ()


########################################
# Find libdl
find_path(libdl_include_dir dlfcn.h /usr/include /usr/local/include)
if (NOT libdl_include_dir)
  message (STATUS "Looking for dlfcn.h - not found")
  BUILD_WARNING ("dlfcn.h not found, plugins will not be supported.")
  set (libdl_include_dir /usr/include)
else (NOT libdl_include_dir)
  message (STATUS "Looking for dlfcn.h - found")
endif (NOT libdl_include_dir)

find_library(libdl_library dl /usr/lib /usr/local/lib)
if (NOT libdl_library)
  message (STATUS "Looking for libdl - not found")
  BUILD_WARNING ("libdl not found, plugins will not be supported.")
else (NOT libdl_library)
  message (STATUS "Looking for libdl - found")
endif (NOT libdl_library)

if (libdl_library AND libdl_include_dir)
  SET (HAVE_DL TRUE)
else (libdl_library AND libdl_include_dir)
  SET (HAVE_DL FALSE)
endif (libdl_library AND libdl_include_dir)
