include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources TopicManager.cc
             Subscriber.cc
             Publisher.cc
             IOManager.cc
             Connection.cc
) 

set (headers TopicManager.hh
             Subscriber.hh
             Subscription.hh
             Publisher.hh
             IOManager.hh
             Connection.hh
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
APPEND_TO_SERVER_HEADERS(${headers})
