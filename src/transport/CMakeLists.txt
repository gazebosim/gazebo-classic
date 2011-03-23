include_directories( 
  ${XML_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/src/msgs
  ${PROJECT_SOURCE_DIR}/src 
)

set (sources TopicManager.cc
             Subscriber.cc
             Publisher.cc
             IOManager.cc
             Connection.cc
             Client.cc
             Server.cc

) 

set (headers TopicManager.hh
             Subscriber.hh
             Publisher.hh
             IOManager.hh
             Connection.hh
             Client.hh
             Server.hh
             CallbackHelper.hh
)

add_library(gazebo_transport SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_transport PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries(gazebo_transport ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})
install ( TARGETS gazebo_transport DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/transport COMPONENT headers)
