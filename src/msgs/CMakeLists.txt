include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (msgs axis.proto
          boxgeom.proto
          camerasensor.proto
          collision.proto
          color.proto
          contact.proto
          contactsensor.proto
          cylindergeom.proto
          factory.proto
          fog.proto
          friction.proto
          geometry.proto
          header.proto
          heightmapgeom.proto
          imagegeom.proto
          inertial.proto
          int.proto
          joint.proto
          light.proto
          link.proto
          material.proto
          meshgeom.proto
          model.proto
          model_v.proto
          packet.proto
          physics.proto
          planegeom.proto
          vector3d.proto
          pose.proto
          publishers.proto
          publish.proto
          quaternion.proto
          raysensor.proto
          request.proto
          response.proto
          scene.proto
          selection.proto
          sensor.proto
          shadows.proto
          spheregeom.proto
          string.proto
          string_v.proto
          subscribe.proto
          surface.proto
          time.proto
          topic_info.proto
          vector2d.proto
          visual.proto
          world_control.proto
          world_stats.proto 
          gui_overlay_config.proto
)

set (sources msgs.cc)
set (headers msgs.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gazebo_msgs SHARED ${PROTO_SRCS} ${sources})
target_link_libraries( gazebo_msgs ${PROTOBUF_LIBRARY} gazebo_sdf_interface)

set (message_headers "" CACHE INTERNAL "Include dirs description")

foreach (hdr ${PROTO_HDRS})
  string (REPLACE "${CMAKE_CURRENT_BINARY_DIR}/" "" hdr ${hdr})
  APPEND_TO_CACHED_STRING(message_headers "Message Types" "#include \"${hdr}\"\n")
endforeach()

configure_file ( ${PROJECT_SOURCE_DIR}/src/msgs/MessageTypes.hh.in ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh )

INSTALL_LIBRARY(gazebo_msgs)
INSTALL_INCLUDES("msgs" ${PROTO_HDRS} ${headers} ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh)


