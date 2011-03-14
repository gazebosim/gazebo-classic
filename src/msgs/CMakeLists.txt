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

file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh "// Automatically Generatated. Do not modify.\n")

foreach (hdr ${PROTO_HDRS})
  string (REPLACE "${CMAKE_CURRENT_BINARY_DIR}/" "" hdr ${hdr})
  file (APPEND ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh "#include \"${hdr}\"\n")
endforeach()

install ( TARGETS gazebo_msgs DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install (FILES ${PROTO_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/msgs COMPONENT headers)

