find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/gazebo.in ${CMAKE_BINARY_DIR}/gazebo.dox @ONLY)
  add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/gazebo.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/gazebo_logo.png ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/search.js ${CMAKE_BINARY_DIR}/doxygen/html/search
    COMMAND make -C ${CMAKE_BINARY_DIR}/doxygen/latex
    COMMAND mv ${CMAKE_BINARY_DIR}/doxygen/latex/refman.pdf ${CMAKE_BINARY_DIR}/doxygen/latex/gazebo-${GAZEBO_VERSION_FULL}.pdf
    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

