set (sources Server.cc 
             Client.cc 
             Iface.cc 
             IfaceFactory.cc 
             FactoryIface.cc 
             SimIface.cc 
             Graphics3dIface.cc
)

set (headers gz.h IfaceFactory.hh)

include_directories(${boost_include_dirs})

if (${ENABLE_BINDINGS})
  add_subdirectory(bindings)
endif (${ENABLE_BINDINGS})

link_directories(${boost_library_dirs})

add_library(gazeboshm SHARED ${sources})
set_target_properties(gazeboshm PROPERTIES OUTPUT_NAME "gazebo" VERSION ${GAZEBO_VERSION})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazeboshm PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})


target_link_libraries( gazeboshm ${boost_libraries})

install (TARGETS gazeboshm DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo)

configure_file(${CMAKE_SOURCE_DIR}/cmake/libgazebo_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/libgazebo.pc @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libgazebo.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig COMPONENT pkgconfig)
