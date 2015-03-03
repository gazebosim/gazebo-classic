macro(ADD_MANPAGE_TARGET)
  # It is not possible add a dependency to target 'install'
  # Run hard-coded 'make man' when 'make install' is invoked
  install(CODE "EXECUTE_PROCESS(COMMAND make man)")
  add_custom_target(man)
endmacro(ADD_MANPAGE_TARGET)

find_program(GZIP gzip)

if (NOT GZIP)
  BUILD_WARNING ("gzip not found, manpages won't be generated")
  macro(roffman MANFILE)
  endmacro(roffman)
else (NOT GZIP)
  message (STATUS "Looking for gzip to generate manpages - found")

  macro(roffman _source _section)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_source}.${_section}.gz
      COMMAND ${GZIP} -c ${CMAKE_CURRENT_SOURCE_DIR}/${_source}.${_section}.roff
        > ${CMAKE_CURRENT_BINARY_DIR}/${_source}.${_section}.gz
    )

    set(MANPAGE_TARGET "man-${_source}")

    add_custom_target(${MANPAGE_TARGET}
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_source}.${_section}.gz)
    add_dependencies(man ${MANPAGE_TARGET})

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${_source}.${_section}.gz
      DESTINATION share/man/man${_section}
    )
  endmacro(roffman _source _section)
endif(NOT GZIP)
