include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})

set (msgs
  altimeter.proto
  axis.proto
  boxgeom.proto
  camerasensor.proto
  camera_cmd.proto
  cessna.proto
  collision.proto
  color.proto
  contact.proto
  contacts.proto
  contactsensor.proto
  cylindergeom.proto
  diagnostics.proto
  distortion.proto
  factory.proto
  fluid.proto
  fog.proto
  friction.proto
  geometry.proto
  gps.proto
  gui_camera.proto
  gui.proto
  gz_string.proto
  gz_string_v.proto
  header.proto
  heightmapgeom.proto
  hydra.proto
  imagegeom.proto
  image.proto
  image_stamped.proto
  images_stamped.proto
  imu.proto
  inertial.proto
  int.proto
  joint.proto
  joint_animation.proto
  joint_cmd.proto
  joint_wrench.proto
  joint_wrench_stamped.proto
  joystick.proto
  laserscan.proto
  laserscan_stamped.proto
  light.proto
  link.proto
  link_data.proto
  log_control.proto
  log_playback_control.proto
  log_playback_stats.proto
  log_status.proto
  logical_camera_image.proto
  logical_camera_sensor.proto
  magnetometer.proto
  material.proto
  meshgeom.proto
  model.proto
  model_configuration.proto
  model_v.proto
  packet.proto
  physics.proto
  planegeom.proto
  pid.proto
  plugin.proto
  pointcloud.proto
  polylinegeom.proto
  pose.proto
  pose_animation.proto
  pose_stamped.proto
  pose_trajectory.proto
  pose_v.proto
  poses_stamped.proto
  projector.proto
  propagation_particle.proto
  propagation_grid.proto
  publishers.proto
  publish.proto
  quaternion.proto
  sonar.proto
  sonar_stamped.proto
  raysensor.proto
  request.proto
  response.proto
  rest_error.proto
  rest_login.proto
  rest_logout.proto
  rest_post.proto
  road.proto
  scene.proto
  selection.proto
  sensor.proto
  server_control.proto
  shadows.proto
  sim_event.proto
  sky.proto
  spheregeom.proto
  spherical_coordinates.proto
  subscribe.proto
  surface.proto
  tactile.proto
  test.proto
  time.proto
  topic_info.proto
  track_visual.proto
  vector2d.proto
  vector3d.proto
  visual.proto
  wireless_node.proto
  wireless_nodes.proto
  world_control.proto
  world_reset.proto
  world_stats.proto
  world_modify.proto
  wrench.proto
  wrench_stamped.proto
)

set (msgs_tests_sources
  msgs_TEST.cc
  MsgFactory_TEST.cc
)
gz_build_tests(${msgs_tests_sources})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(gazebomsgs_out generator/GazeboGenerator.cc generator/gazebo_generator.cc)
target_link_libraries(gazebomsgs_out ${GZ_PROTOBUF_LIBRARY} ${GZ_PROTOBUF_PROTOC_LIBRARY})
if (UNIX)
  target_link_libraries(gazebomsgs_out pthread)
endif()

set (sources msgs.cc MsgFactory.cc)
set (headers msgs.hh MsgFactory.hh)

###########################################################
# Append str to a string property of a target.
# target: string: target name.
# property: name of target’s property. e.g: COMPILE_FLAGS, or LINK_FLAGS
# str: string: string to be appended to the property
macro(my_append_target_property target property str)
  get_target_property(current_property ${target} ${property})
  if(NOT current_property) # property non-existent or empty
      set_target_properties(${target} PROPERTIES GENERATED TRUE ${property} ${str})
  else()
      set_target_properties(${target} PROPERTIES ${property} "${current_property} ${str}")
  endif()
endmacro(my_append_target_property)

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(FIL ${msgs})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)

  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

  # Need to add .exe to executable in windows
  if (WIN32)
     set(gazebomsgs_out_binary "gazebomsgs_out.exe")
  else()
     set(gazebomsgs_out_binary "gazebomsgs_out")
  endif()

  add_custom_command(
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --plugin=protoc-gen-gazebomsgs=${CMAKE_CURRENT_BINARY_DIR}/${gazebomsgs_out_binary} --cpp_out=dllexport_decl=GZ_MSGS_VISIBLE:${CMAKE_CURRENT_BINARY_DIR} --gazebomsgs_out=${CMAKE_CURRENT_BINARY_DIR} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL} gazebomsgs_out
    COMMENT "Running C++ protocol buffer compiler on ${FIL}"
    VERBATIM )
endforeach()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_MSGS")

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)
gz_add_library(gazebo_msgs ${PROTO_SRCS} ${sources})
if (UNIX)
  # Need to add default visibility to gazebo_msgs
  get_target_property(current_property gazebo_msgs COMPILE_FLAGS)
  if(NOT current_property) # property non-existent or empty
    set_target_properties(gazebo_msgs PROPERTIES GENERATED TRUE COMPILE_FLAGS "-fvisibility=default")
  else()
    set_target_properties(gazebo_msgs PROPERTIES COMPILE_FLAGS "${current_property} -fvisibility=default")
  endif()
endif()

target_link_libraries(gazebo_msgs
  gazebo_common
  gazebo_math
  ${GZ_PROTOBUF_LIBRARY}
)

add_dependencies(gazebo_msgs gazebomsgs_out)

set (message_headers "" CACHE INTERNAL "Include dirs description")

foreach (hdr ${PROTO_HDRS})
  string (REPLACE "${CMAKE_CURRENT_BINARY_DIR}/" "" hdr ${hdr})
  APPEND_TO_CACHED_STRING(message_headers
    "Message Types" "#include \"${hdr}\"\n")
endforeach()
configure_file (${PROJECT_SOURCE_DIR}/gazebo/msgs/MessageTypes.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh )

gz_install_library(gazebo_msgs)
gz_install_includes("msgs" ${PROTO_HDRS} ${headers} ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh)
gz_install_includes("msgs/proto" ${msgs})
