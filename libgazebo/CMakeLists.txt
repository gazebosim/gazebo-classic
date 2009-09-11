SET (sources Server.cc 
             Client.cc 
             Iface.cc 
             IfaceFactory.cc 
             SimIface.cc 
             Graphics3dIface.cc
)

SET (headers gazebo.h IfaceFactory.hh)

INCLUDE_DIRECTORIES(
  ${boost_include_dirs}
)

IF (${ENABLE_BINDINGS})
  ADD_SUBDIRECTORY(bindings)
ENDIF (${ENABLE_BINDINGS})

LINK_DIRECTORIES(${boost_library_dirs})

ADD_LIBRARY(gazeboshm SHARED ${sources})
SET_TARGET_PROPERTIES(gazeboshm PROPERTIES OUTPUT_NAME "gazebo" VERSION ${GAZEBO_VERSION})

TARGET_LINK_LIBRARIES( gazeboshm ${boost_libraries})

INSTALL (TARGETS gazeboshm DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
INSTALL (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/libgazebo_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/libgazebo.pc @ONLY)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/libgazebo.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig COMPONENT pkgconfig)
