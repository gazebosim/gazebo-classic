include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (msgs color.proto
          header.proto
          factory.proto
          light.proto
          plane.proto
          point.proto
          pose.proto
          quaternion.proto
          time.proto
          visual.proto
          selection.proto
          shadows.proto
          scene.proto
          fog.proto
          int.proto
          string.proto
          packet.proto
          publish.proto
          subscribe.proto
          publishers.proto
          request.proto
          topic_info.proto
          world_stats.proto
          world_control.proto
          entity.proto
)

set (sources msgs.cc)
set (headers msgs.h)


include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gazebo_msgs ${PROTO_SRCS} ${sources})
target_link_libraries( gazebo_msgs ${PROTOBUF_LIBRARY})

set (message_headers "" CACHE INTERNAL "Include dirs description")

foreach (hdr ${PROTO_HDRS})
  string (REPLACE "${CMAKE_CURRENT_BINARY_DIR}/" "" hdr ${hdr})
  APPEND_TO_CACHED_STRING(message_headers "Message Types" "#include \"${hdr}\"\n")
endforeach()

configure_file ( ${PROJECT_SOURCE_DIR}/src/msgs/MessageTypes.hh.in ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh )

INSTALL_LIBRARY(gazebo_msgs)
INSTALL_INCLUDES("msgs" ${PROTO_HDRS} ${headers} ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh)
