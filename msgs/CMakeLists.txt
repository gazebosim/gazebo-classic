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
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gazebo_msgs ${PROTO_SRCS})
