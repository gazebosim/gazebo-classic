find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/gazebo.in
                 ${CMAKE_BINARY_DIR}/gazebo.dox @ONLY)

  configure_file(${CMAKE_SOURCE_DIR}/doc/gazebo_messages.in
                 ${CMAKE_BINARY_DIR}/gazebo_messages.dox @ONLY)

  add_custom_target(doc

    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/gazebo.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/gazebo_logo.png
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/search.js
            ${CMAKE_BINARY_DIR}/doxygen/html/search

    # Generate the messages documentation
    COMMAND doxygen ${CMAKE_BINARY_DIR}/gazebo_messages.dox

    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

