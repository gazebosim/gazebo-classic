include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (msgs axis.proto
          boxgeom.proto
          camerasensor.proto
          collision.proto
          color.proto
          contact.proto
          contacts.proto
          contactsensor.proto
          cylindergeom.proto
          factory.proto
          fog.proto
          friction.proto
          geometry.proto
          header.proto
          heightmapgeom.proto
          imagegeom.proto
          image.proto
          inertial.proto
          int.proto
          joint.proto
          laserscan.proto
          light.proto
          body.proto
          material.proto
          meshgeom.proto
          model.proto
          model_v.proto
          packet.proto
          physics.proto
          planegeom.proto
          pid.proto
          pose.proto
          projector.proto
          publishers.proto
          publish.proto
          quaternion.proto
          raysensor.proto
          request.proto
          response.proto
          scene.proto
          selection.proto
          sensor.proto
          server_control.proto
          shadows.proto
          spheregeom.proto
          string.proto
          string_v.proto
          subscribe.proto
          surface.proto
          time.proto
          topic_info.proto
          vector2d.proto
          vector3d.proto
          visual.proto
          world_control.proto
          world_stats.proto 
          world_modify.proto 
          gui_overlay_config.proto
          gui_camera.proto
          gui.proto
          track_visual.proto
          test.proto
          pose_animation.proto
          joint_animation.proto
          joint_cmd.proto
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

gz_add_executable(gazebomsgs_out generator/GazeboGenerator.cc generator/gazebo_generator.cc)
target_link_libraries(gazebomsgs_out ${PROTOBUF_LIBRARY} ${PROTOBUF_PROTOC_LIBRARY} pthread)

set (sources msgs.cc)
set (headers msgs.h)

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(FIL ${msgs})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)

  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

  add_custom_command(TARGET gazebomsgs_out POST_BUILD
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --plugin=protoc-gen-gazebomsgs=${CMAKE_CURRENT_BINARY_DIR}/gazebomsgs_out --cpp_out ${CMAKE_CURRENT_BINARY_DIR} --gazebomsgs_out ${CMAKE_CURRENT_BINARY_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${FIL}"
    VERBATIM )
endforeach()

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

gz_add_library(gazebo_msgs ${PROTO_SRCS} ${sources})
target_link_libraries( gazebo_msgs ${PROTOBUF_LIBRARY} gazebo_sdf_interface)
add_dependencies(gazebo_msgs gazebomsgs_out)

set (message_headers "" CACHE INTERNAL "Include dirs description")

foreach (hdr ${PROTO_HDRS})
  string (REPLACE "${CMAKE_CURRENT_BINARY_DIR}/" "" hdr ${hdr})
  APPEND_TO_CACHED_STRING(message_headers "Message Types" "#include \"${hdr}\"\n")
endforeach()

configure_file ( ${PROJECT_SOURCE_DIR}/src/msgs/MessageTypes.hh.in ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh )

INSTALL_LIBRARY(gazebo_msgs)
INSTALL_INCLUDES("msgs" ${PROTO_HDRS} ${headers} ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh)


