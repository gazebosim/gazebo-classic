include_directories( 
  ${PROJECT_BINARY_DIR}/src/msgs
  ${PROJECT_SOURCE_DIR}/src 
)

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

add_library(gazebo_transport SHARED ${sources})

target_link_libraries(gazebo_transport gazebo_common 
                                       gazebo_msgs 
                                       ${boost_libraries})

INSTALL_LIBRARY(gazebo_transport)
INSTALL_INCLUDES("transport" ${headers})
