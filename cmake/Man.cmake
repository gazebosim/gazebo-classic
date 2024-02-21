macro(ADD_MANPAGE_TARGET)
  # It is not possible add a dependency to target 'install'
  # Run hard-coded 'cmake --build . --target man' when the target install is built
  install(CODE "EXECUTE_PROCESS(COMMAND cmake --build . --target man)")
  add_custom_target(man)
endmacro(ADD_MANPAGE_TARGET)

find_program(GZIP gzip)

if (NOT GZIP)
  BUILD_WARNING ("gzip not found, manpages won't be generated")
  macro(roffman MANFILE)
  endmacro(roffman)
else (NOT GZIP)
  message (STATUS "Looking for gzip to generate manpages - found")

  # macro can also be called with a third argument that contains a modified name
  # of the manpage to be installed.
  macro(roffman _source _section)
    set(_extra_macro_args ${ARGN})
    list(LENGTH _extra_macro_args _num_extra_macro_args)

    if (_num_extra_macro_args EQUAL 0)
      set(_destination _source)
    elseif(_num_extra_macro_args EQUAL 1)
      set(_destination ${_extra_macro_args})
    else()
      message(FATAL_ERROR "roffman macro called with unexpected number of extra arguments '${ARGN}'")
    endif()

    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_destination}.${_section}.gz
      COMMAND ${GZIP} -c ${CMAKE_CURRENT_SOURCE_DIR}/${_source}.${_section}.roff
        > ${CMAKE_CURRENT_BINARY_DIR}/${_destination}.${_section}.gz
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${_source}.${_section}.roff
    )

    set(MANPAGE_TARGET "man-${_source}")

    add_custom_target(${MANPAGE_TARGET}
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_destination}.${_section}.gz)
    add_dependencies(man ${MANPAGE_TARGET})

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${_destination}.${_section}.gz
      DESTINATION share/man/man${_section}
    )
  endmacro(roffman _source _section)
endif(NOT GZIP)
