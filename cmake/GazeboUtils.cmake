
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
# Append sources to the server sources
MACRO (APPEND_TO_SERVER_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazeboserver_sources 
                          ${gazeboserver_sources_desc}                   
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SERVER_SOURCES)

###############################################################################
# Reset lists
MACRO (GAZEBOSERVER_RESET_LISTS)
  SET (gazeboserver_sources "" CACHE INTERNAL 
       ${gazeboserver_sources_desc} FORCE)
  SET (gazeboserver_include_dirs "" CACHE INTERNAL 
       ${gazeboserver_include_dirs_desc} FORCE)
  SET (gazeboserver_link_dirs "" CACHE INTERNAL 
       ${gazeboserver_link_dirs_desc} FORCE)
  SET (gazeboserver_link_libs "" CACHE INTERNAL 
       ${gazeboserver_link_libs_desc} FORCE)
ENDMACRO (GAZEBOSERVER_RESET_LISTS)
