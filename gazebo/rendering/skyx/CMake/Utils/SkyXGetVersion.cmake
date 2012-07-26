#-------------------------------------------------------------------
# This file is part of the CMake build system for SKYX
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

macro(get_preprocessor_entry CONTENTS KEYWORD VARIABLE)
  string(REGEX MATCH
    "# *define +${KEYWORD} +((\"([^\n]*)\")|([^ \n]*))"
    PREPROC_TEMP_VAR
    ${${CONTENTS}}
  )
  if (CMAKE_MATCH_3)
    set(${VARIABLE} ${CMAKE_MATCH_3})
  else ()
    set(${VARIABLE} ${CMAKE_MATCH_4})
  endif ()
endmacro()

macro(skyx_get_version HEADER)
	file(READ ${HEADER} TEMP_VAR_CONTENTS)
	get_preprocessor_entry(TEMP_VAR_CONTENTS SKYX_VERSION_MAJOR SKYX_VERSION_MAJOR)
	get_preprocessor_entry(TEMP_VAR_CONTENTS SKYX_VERSION_MINOR SKYX_VERSION_MINOR)
	get_preprocessor_entry(TEMP_VAR_CONTENTS SKYX_VERSION_PATCH SKYX_VERSION_PATCH)
	set(SKYX_VERSION "${SKYX_VERSION_MAJOR}.${SKYX_VERSION_MINOR}.${SKYX_VERSION_PATCH}")
endmacro()
