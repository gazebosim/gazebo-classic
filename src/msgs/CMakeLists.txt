include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (msgs color.proto
          header.proto
          insert_model.proto
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
)


include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gazebo_msgs ${PROTO_SRCS})

set (message_headers "" CACHE INTERNAL "Include dirs description")

foreach (hdr ${PROTO_HDRS})
  string (REPLACE "${CMAKE_CURRENT_BINARY_DIR}/" "" hdr ${hdr})
  APPEND_TO_CACHED_STRING(message_headers "Message Types" "#include \"${hdr}\"\n")
endforeach()

configure_file ( ${PROJECT_SOURCE_DIR}/src/msgs/MessageTypes.hh.in ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh )

install (TARGETS gazebo_msgs DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install (FILES ${PROTO_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/msgs COMPONENT headers)

