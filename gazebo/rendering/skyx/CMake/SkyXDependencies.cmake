#-------------------------------------------------------------------
# This file is part of the CMake build system for SKYX
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

#######################################################################
# Find all necessary and optional SKYX dependencies
#######################################################################

# SKYX_DEPENDENCIES_DIR can be used to specify a single base
# folder where the required dependencies may be found.
set(SKYX_DEPENDENCIES_DIR "" CACHE PATH "Path to prebuilt SKYX dependencies")
include(SkyXFindPkgMacros)

set(SKYX_DEP_SEARCH_PATH
  ${SKYX_DEPENDENCIES_DIR}
  ${ENV_SKYX_DEPENDENCIES_DIR}
  ${ENV_OGRE_HOME}
)
message(STATUS "Search path: ${SKYX_DEP_SEARCH_PATH}")

# Set hardcoded path guesses for various platforms
if (UNIX)
  set(SKYX_DEP_SEARCH_PATH ${SKYX_DEP_SEARCH_PATH}
                           /usr/local)
endif ()

# give guesses as hints to the find_package calls
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${SKYX_DEP_SEARCH_PATH})
set(CMAKE_FRAMEWORK_PATH ${CMAKE_FRAMEWORK_PATH} ${SKYX_DEP_SEARCH_PATH})


#######################################################################
# Core dependencies
#######################################################################

# Find Boost, you can comment those lines if Ogre was not compiled using boost threads.
set(Boost_USE_STATIC_LIBS TRUE)
set(Boost_ADDITIONAL_VERSIONS "1.47.0" "1.47" "1.46.0" "1.46" "1.45.0" "1.45" "1.44.0" "1.44" "1.43.0" "1.43" "1.42.0" "1.42" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
# Uncomment bellow if Ogre was compiled with boost threading
#set(SKYX_BOOST_COMPONENTS thread date_time)
find_package(Boost COMPONENTS ${SKYX_BOOST_COMPONENTS} QUIET)
if (NOT Boost_FOUND)
  # Try again with the other type of libs
  if(Boost_USE_STATIC_LIBS)
    set(Boost_USE_STATIC_LIBS)
  else()
    set(Boost_USE_STATIC_LIBS ON)
  endif()
  find_package(Boost COMPONENTS ${SKYX_BOOST_COMPONENTS} QUIET)
endif()
macro_log_feature(Boost_FOUND "boost" "Boost (general)" "http://boost.org" TRUE "" "")
macro_log_feature(Boost_THREAD_FOUND "boost-thread" "Used for threading support" "http://boost.org" FALSE "" "")
macro_log_feature(Boost_DATE_TIME_FOUND "boost-date_time" "Used for threading support" "http://boost.org" FALSE "" "")

# Find Ogre 3D, plus terrain and paging components
find_package(OGRE)
macro_log_feature(OGRE_FOUND "OGRE" "3D library needed for the OgreGraphics plugin" "http://" TRUE "" "")
#set (OGRE_INCLUDE_DIRS "/home/nkoenig/local/include/OGRE")
#set (OGRE_LIBRARY_DIRS "/home/nkoenig/local/lib/OGRE /home/nkoenig/local/lib")

#######################################################################
# Tools dependencies
#######################################################################

# Find Doxygen
find_package(Doxygen)
macro_log_feature(DOXYGEN_FOUND "Doxygen" "Tool for building API documentation" "http://doxygen.org" FALSE "" "")

#######################################################################
# Samples dependencies (comment if not needed)
#######################################################################

# Find OIS
find_package(OIS)
macro_log_feature(OIS_FOUND "OIS" "Input library needed for the OISInput plugin" "http://sourceforge.net/projects/wgois" TRUE "" "")

#######################################################################
# All dependencies were checked
#######################################################################

# Display results, terminate if anything required is missing
MACRO_DISPLAY_FEATURE_LOG()

# Add library and include paths from the dependencies
include_directories(
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${OGRE_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

