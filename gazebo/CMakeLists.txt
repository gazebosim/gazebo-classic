include_directories(SYSTEM
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${TBB_INCLUDEDIR}
  ${tinyxml_INCLUDE_DIRS}
)

link_directories(
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${PROJECT_BINARY_DIR}/test
  ${TBB_LIBRARY_DIR}
  ${CURL_LIBDIR}
)

if (WIN32)
  include_directories(${libdl_include_dir})
endif()

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

if (WIN32)
 add_definitions(-DBUILDING_DLL)
endif()

add_subdirectory(msgs)
add_subdirectory(math)
add_subdirectory(transport)
add_subdirectory(common)
add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(physics)
add_subdirectory(sensors)
add_subdirectory(util)
add_subdirectory(test)

add_dependencies(gazebo_physics gazebo_msgs)
add_dependencies(gazebo_physics_ode gazebo_msgs)
add_dependencies(gazebo_gui gazebo_msgs)
add_dependencies(gazebo_rendering gazebo_msgs)
add_dependencies(gazebo_sensors gazebo_rendering)

if(HAVE_BULLET)
  add_dependencies(gazebo_physics_bullet gazebo_msgs)
endif()
if (HAVE_SIMBODY)
  add_dependencies(gazebo_physics_simbody gazebo_msgs)
endif()
if(HAVE_DART)
  add_dependencies(gazebo_physics_dart gazebo_msgs)
endif()


gz_add_executable(gzserver server_main.cc)

target_link_libraries(gzserver
  libgazebo
  gazebo_common
  gazebo_util
  gazebo_transport
  gazebo_physics
  gazebo_sensors
  gazebo_rendering
  gazebo_msgs
  # Extra libs needed when linking statically (e.g., on WIN32)
  gazebo_physics_ode
  ${freeimage_LIBRARIES}
  ${TBB_LIBRARIES}
)

if (HAVE_SIMBODY)
  target_link_libraries(gzserver gazebo_physics_simbody)
endif()

if (HAVE_DART)
  target_link_libraries(gzserver gazebo_physics_dart)
endif()

if (HAVE_BULLET)
  target_link_libraries(gzserver gazebo_physics_bullet)
endif()

if (UNIX)
  target_link_libraries(gzserver pthread)
endif()

if ("${CMAKE_BUILD_TYPE}" STREQUAL "CHECK")
  target_link_libraries(gzserver gcov)
endif()

gz_install_executable(gzserver)
manpage(gzserver 1)

# gazebo executable doesn't yet work on Windows
if (NOT WIN32)
  gz_add_executable(gazebo gazebo_main.cc)
  target_link_libraries(gazebo
    libgazebo
    libgazebo_client
    gazebo_common
    gazebo_util
    gazebo_transport
    gazebo_physics
    gazebo_sensors
    gazebo_rendering
    gazebo_msgs
    gazebo_gui
  )

  gz_install_executable(gazebo)
  manpage(gazebo 1)
endif()

gz_add_library(libgazebo Server.cc Master.cc gazebo.cc gazebo_shared.cc)
set_target_properties(libgazebo PROPERTIES OUTPUT_NAME "gazebo")

target_link_libraries(libgazebo
  gazebo_common
  gazebo_util
  gazebo_transport
  gazebo_physics
  gazebo_sensors
  gazebo_rendering
  gazebo_msgs
  gazebo_math
)
if (UNIX)
  target_link_libraries(libgazebo pthread)
endif()

add_dependencies(libgazebo gazebo_msgs)
gz_install_library(libgazebo)

gz_add_library(libgazebo_client gazebo_client.cc gazebo_shared.cc)
set_target_properties(libgazebo_client PROPERTIES OUTPUT_NAME "gazebo_client")
target_link_libraries(libgazebo_client
  gazebo_common
  gazebo_transport
  gazebo_util
)
if (UNIX)
  target_link_libraries(libgazebo_client pthread)
endif()
gz_install_library(libgazebo_client)

set(headers
  gazebo_client.hh
  gazebo_core.hh
  gazebo.hh
  Master.hh
  Server.hh
)
gz_install_includes("" ${headers})
