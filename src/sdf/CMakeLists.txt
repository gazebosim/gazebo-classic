add_subdirectory(interface)
add_subdirectory(parser)
add_subdirectory(parser_deprecated)

set (headers sdf.h sdf_parser.h)

INSTALL_INCLUDES("sdf" ${headers})

INSTALL(FILES sdf_format.xml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/)
