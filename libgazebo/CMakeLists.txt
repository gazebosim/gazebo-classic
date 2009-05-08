########################################
# Find Boost, if not specified manually
IF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )
  MESSAGE(STATUS "HERE")
  INCLUDE (FindBoost)
  FIND_PACKAGE( Boost 1.34.1 COMPONENTS thread signals)
  IF (NOT Boost_FOUND)
    MESSAGE (FATAL_ERROR "Boost thread and signals not found")
    SET (BUILD_GAZEBO OFF CACHE BOOL "Build Gazebo" FORCE)
  ENDIF (NOT Boost_FOUND)


  LIST_TO_STRING(tmp "${Boost_INCLUDE_DIRS}")
  SET (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  LIST_TO_STRING(tmp "${Boost_LIBRARY_DIRS}")
  SET (boost_library_dirs ${Boost_LIBRARY_DIRS} CACHE STRING 
    "Boost library paths. Use this to override automatic detection." FORCE)

  LIST_TO_STRING(tmp "${Boost_LIBRARIES}")
  SET (boost_libraries ${Boost_LIBRARIES} CACHE STRING 
    "Boost libraries. Use this to override automatic detection." FORCE)
ENDIF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )


SET (sources Server.cc 
             Client.cc 
             Iface.cc 
             IfaceFactory.cc 
             SimIface.cc 
             Graphics3dIface.cc
)

SET (headers gazebo.h IfaceFactory.hh)

ADD_LIBRARY(gazeboshm SHARED ${sources})
TARGET_LINK_LIBRARIES( gazeboshm ${boost_libraries})

INSTALL (TARGETS gazeboshm DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
INSTALL (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/libgazeboshm_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/libgazeboshm.pc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/libgazeboshm.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig COMPONENT pkgconfig)
