SET (sources Server.cc 
             Client.cc 
             Iface.cc 
             IfaceFactory.cc 
             SimIface.cc 
             Graphics3dIface.cc
)

SET (headers gazebo.h IfaceFactory.hh)

LINK_DIRECTORIES(${boost_library_dirs})

ADD_LIBRARY(gazeboshm SHARED ${sources})

TARGET_LINK_LIBRARIES( gazeboshm ${boost_libraries})

INSTALL (TARGETS gazeboshm DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
INSTALL (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/libgazeboshm_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/libgazeboshm.pc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/libgazeboshm.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig COMPONENT pkgconfig)
