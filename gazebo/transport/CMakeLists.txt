include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${TBB_INCLUDEDIR})

set (sources
  CallbackHelper.cc
  Connection.cc
  ConnectionManager.cc
  IOManager.cc
  Node.cc
  Publication.cc
  PublicationTransport.cc
  Publisher.cc
  Subscriber.cc
  SubscriptionTransport.cc
  TopicManager.cc
  TransportIface.cc
)

set (headers
  CallbackHelper.hh
  Connection.hh
  ConnectionManager.hh
  IOManager.hh
  Node.hh
  Publication.hh
  Publisher.hh
  PublicationTransport.hh
  SubscribeOptions.hh
  Subscriber.hh
  SubscriptionTransport.hh
  TopicManager.hh
  TransportIface.hh
  TransportTypes.hh
)

set (transport_headers "" CACHE INTERNAL "transport headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(transport_headers "Transport Headers"
    "#include \"gazebo/transport/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/transport.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/transport.hh )

set_property(
  SOURCE TransportIface.cc
  PROPERTY COMPILE_DEFINITIONS
  GAZEBO_DEFAULT_MASTER_HOST="${GAZEBO_DEFAULT_MASTER_HOST}"
  GAZEBO_DEFAULT_MASTER_PORT=${GAZEBO_DEFAULT_MASTER_PORT}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_TRANSPORT")
if (WIN32)
  # 0x601 = Windows 7.  TBB needs to know this.
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_WIN32_WINNT=0x601")
endif ()

gz_add_library(gazebo_transport ${sources})
target_link_libraries(gazebo_transport
  gazebo_common
  gazebo_msgs
  ${IGNITION-MATH_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES}
)
if (WIN32)
  target_link_libraries(gazebo_transport ws2_32 Iphlpapi)
endif()

gz_install_library(gazebo_transport)
gz_install_includes("transport" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/transport.hh)

# unit tests
set (gtest_sources
  Connection_TEST.cc
)
gz_build_tests(${gtest_sources})
