################################################################################
#APPEND_TO_CACHED_STRING(_string _cacheDesc [items...])
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_STRING _string _cacheDesc)
  FOREACH (newItem ${ARGN})
    SET (${_string} "${${_string}} ${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  ENDFOREACH (newItem ${ARGN})
  #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (APPEND_TO_CACHED_STRING)

################################################################################
# APPEND_TO_CACHED_LIST (_list _cacheDesc [items...]
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_LIST _list _cacheDesc)
  SET (tempList ${${_list}})
  FOREACH (newItem ${ARGN})
    LIST (APPEND tempList ${newItem})
  ENDFOREACH (newItem ${newItem})
  SET (${_list} ${tempList} CACHE INTERNAL ${_cacheDesc} FORCE)
ENDMACRO(APPEND_TO_CACHED_LIST)

###############################################################################
# Append sources to the server sources list
MACRO (APPEND_TO_SERVER_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazeboserver_sources
                          ${gazeboserver_sources_desc}
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SERVER_SOURCES)

###############################################################################
# Append headers to the server headers list
MACRO (APPEND_TO_SERVER_HEADERS)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazeboserver_headers
                          ${gazeboserver_headers_desc}
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
    APPEND_TO_CACHED_LIST(gazeboserver_headers_nopath
                          "gazeboserver_headers_nopath"
                          ${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SERVER_HEADERS)

###############################################################################
# Append sources to the sensor sources list
MACRO (APPEND_TO_SENSOR_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazebosensor_sources
                          ${gazebosensor_sources_desc}
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SENSOR_SOURCES)

###############################################################################
# Append sources to the controller sources list
MACRO (APPEND_TO_CONTROLLER_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazebocontroller_sources
                          ${gazebocontroller_sources_desc}
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_CONTROLLER_SOURCES)


#################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
MACRO (LIST_TO_STRING _string _list)
    SET (${_string})
    FOREACH (_item ${_list})
      SET (${_string} "${${_string}} ${_item}")
    ENDFOREACH (_item)
    #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (LIST_TO_STRING)

#################################################
# BUILD ERROR macro
macro (BUILD_ERROR)
  foreach (str ${ARGN})
    SET (msg "\t${str}")
    MESSAGE (STATUS ${msg})
    APPEND_TO_CACHED_LIST(build_errors "build errors" ${msg})
  endforeach ()
endmacro (BUILD_ERROR)

#################################################
# BUILD WARNING macro
macro (BUILD_WARNING)
  foreach (str ${ARGN})
    SET (msg "\t${str}" )
    MESSAGE (STATUS ${msg} )
    APPEND_TO_CACHED_LIST(build_warnings "build warning" ${msg})
  endforeach (str ${ARGN})
endmacro (BUILD_WARNING)

#################################################
macro (gz_add_library _name)
  # Not defining STATIC or SHARED will use BUILD_SHARED_LIBS variable
  add_library(${_name} ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
endmacro ()

#################################################
macro (gz_add_executable _name)
  add_executable(${_name} ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
endmacro ()


#################################################
macro (gz_install_includes _subdir)
  install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT headers)
endmacro()

#################################################
macro (gz_install_library _name)
  set_target_properties(${_name} PROPERTIES SOVERSION ${GAZEBO_MAJOR_VERSION} VERSION ${GAZEBO_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${LIB_INSTALL_DIR} COMPONENT shlib)
endmacro ()

#################################################
macro (gz_install_executable _name)
  set_target_properties(${_name} PROPERTIES VERSION ${GAZEBO_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${BIN_INSTALL_DIR})
endmacro ()

#################################################
macro (gz_setup_unix)
    # Using dynamic linking in UNIX by default
    set(BUILD_SHARED_LIBS TRUE)
    add_definitions(-DBUILDING_STATIC_LIBS)
endmacro()

#################################################
macro (gz_setup_windows)
    # Using static linking in Windows by default
    set(BUILD_SHARED_LIBS FALSE)
endmacro()

#################################################
macro (gz_setup_apple)
  # NOTE MacOSX provides different system versions than CMake is parsing.
  #      The following table lists the most recent OSX versions
  #     9.x.x = Mac OSX Leopard (10.5)
  #    10.x.x = Mac OSX Snow Leopard (10.6)
  #    11.x.x = Mac OSX Lion (10.7)
  #    12.x.x = Mac OSX Mountain Lion (10.8)
  if (${CMAKE_SYSTEM_VERSION} LESS 10)
    add_definitions(-DMAC_OS_X_VERSION=1050)
  elseif (${CMAKE_SYSTEM_VERSION} GREATER 10 AND ${CMAKE_SYSTEM_VERSION} LESS 11)
    add_definitions(-DMAC_OS_X_VERSION=1060)
  elseif (${CMAKE_SYSTEM_VERSION} GREATER 11 AND ${CMAKE_SYSTEM_VERSION} LESS 12)
    add_definitions(-DMAC_OS_X_VERSION=1070)
  elseif (${CMAKE_SYSTEM_VERSION} GREATER 12 OR ${CMAKE_SYSTEM_VERSION} EQUAL 12)
    add_definitions(-DMAC_OS_X_VERSION=1080)
  else ()
    add_definitions(-DMAC_OS_X_VERSION=0)
  endif ()

  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined -Wl,dynamic_lookup")
endmacro()

# This should be migrated to more fine control solution based on set_property APPEND
# directories. It's present on cmake 2.8.8 while precise version is 2.8.7
link_directories(${PROJECT_BINARY_DIR}/test)
include_directories("${PROJECT_SOURCE_DIR}/test/gtest/include")

#################################################
# Enable tests compilation by default
if (NOT DEFINED ENABLE_TESTS_COMPILATION)
  set (ENABLE_TESTS_COMPILATION True)
endif()

# Define testing macros as empty and redefine them if support is found and 
# ENABLE_TESTS_COMPILATION is set to true
macro (gz_build_tests)
endmacro()
macro (gz_build_qt_tests)
endmacro()
macro (gz_build_display_tests)
endmacro()
macro (gz_build_dri_tests)
endmacro()

if (ENABLE_TESTS_COMPILATION)
  include (${gazebo_cmake_dir}/GazeboTestUtils.cmake)
endif()

#################################################
# Macro to setup supported compiler flags
# Based on work of Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST. 
include(CheckCXXCompilerFlag)

macro(filter_valid_compiler_flags) 
  foreach(flag ${ARGN})
    CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
    if(${R${flag}})
      set(VALID_CXX_FLAGS "${VALID_CXX_FLAGS} ${flag}")
    endif()
  endforeach()
endmacro()
