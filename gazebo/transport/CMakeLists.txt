include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources TopicManager.cc
             Subscriber.cc
             Publisher.cc
             IOManager.cc
             Connection.cc
             ConnectionManager.cc
             Publication.cc
             SubscriptionTransport.cc
             PublicationTransport.cc
             Transport.cc
             Node.cc
) 

set (headers TopicManager.hh
             Subscriber.hh
             Publisher.hh
             IOManager.hh
             Connection.hh
             ConnectionManager.hh
             CallbackHelper.hh
             Publication.hh
             SubscriptionTransport.hh
             PublicationTransport.hh
             SubscribeOptions.hh
             Transport.hh
             TransportTypes.hh
             Node.hh
)

set (transport_headers "" CACHE INTERNAL "transport headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(transport_headers "Transport Headers"
    "#include \"transport/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/transport.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/transport.hh )


gz_add_library(gazebo_transport ${sources})
target_link_libraries(gazebo_transport gazebo_common 
                                       gazebo_msgs 
                                       ${Boost_LIBRARIES})

gz_install_library(gazebo_transport)
gz_install_includes("transport" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/transport.hh)
