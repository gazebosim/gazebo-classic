
################################################################################
#APPEND_TO_CACHED_STRING(_string _cacheDesc [items...])
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_STRING _string _cacheDesc)
  FOREACH (newItem ${ARGN})
    SET (${_string} "${${_string}} ${newItem}" CACHE STRING ${_cacheDesc} FORCE)
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


###############################################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
MACRO (LIST_TO_STRING _string _list)
    SET (${_string})
    FOREACH (_item ${_list})
      SET (${_string} "${${_string}} ${_item}")
    ENDFOREACH (_item)
    #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (LIST_TO_STRING)

macro (BUILD_ERROR)
  foreach (str ${ARGN})
    SET (msg "ERROR: ${str}" )
    MESSAGE (STATUS ${msg} )
    APPEND_TO_CACHED_LIST(build_errors "build errors" ${msg})
  endforeach (str ${ARGN})
endmacro (BUILD_ERROR)

###############################################################################
# Reset lists
MACRO (GAZEBOSERVER_RESET_LISTS)
  SET (gazeboserver_sources "" CACHE INTERNAL 
       ${gazeboserver_sources_desc} FORCE)
  SET (gazeboserver_headers "" CACHE INTERNAL 
       ${gazeboserver_sources_desc} FORCE)
  SET (gazeboserver_include_dirs "" CACHE INTERNAL 
       ${gazeboserver_include_dirs_desc} FORCE)
  SET (gazeboserver_link_dirs "" CACHE INTERNAL 
       ${gazeboserver_link_dirs_desc} FORCE)
  SET (gazeboserver_link_libs "" CACHE INTERNAL 
       ${gazeboserver_link_libs_desc} FORCE)
  SET (gazeboserver_ldflags "" CACHE INTERNAL 
       ${gazeboserver_ldflags_desc} FORCE)
  SET (gazeboserver_cflags "" CACHE INTERNAL 
       ${gazeboserver_cflags_desc} FORCE)
  SET (gazebosensor_sources "" CACHE INTERNAL 
       ${gazebosensor_sources_desc} FORCE)
  SET (gazebocontroller_sources "" CACHE INTERNAL 
       ${gazebocontroller_sources_desc} FORCE)
  set (bullet_link_libs "" CACHE INTERNAL 
       ${bullet_link_libs_desc} FORCE)
ENDMACRO (GAZEBOSERVER_RESET_LISTS)
