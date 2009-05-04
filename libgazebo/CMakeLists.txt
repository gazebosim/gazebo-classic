INCLUDE (FindBoost)

FIND_PACKAGE( Boost 1.34.1 COMPONENTS thread)

SET (sources Server.cc 
             Client.cc 
             Iface.cc 
             IfaceFactory.cc 
             SimIface.cc 
             Graphics3dIface.cc
)

ADD_LIBRARY(gazeboshm SHARED ${sources})
INSTALL (TARGETS gazeboshm DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/libgazeboshm_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/libgazeboshm.pc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/libgazeboshm.pc DESTINATION lib/pkgconfig COMPONENT pkgconfig)
