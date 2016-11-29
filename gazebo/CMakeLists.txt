include_directories(SYSTEM
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${TBB_INCLUDEDIR}
  ${tinyxml_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}
)

link_directories(
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${PROJECT_BINARY_DIR}/test
  ${TBB_LIBRARY_DIR}
  ${CURL_LIBDIR}
  ${IGNITION-MATH_LIBRARY_DIRS}
)

if (WIN32)
  include_directories(${libdl_include_dir})
endif()

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

add_definitions(-DBUILDING_DLL)

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
add_dependencies(gazebo_gui gazebo_msgs)
add_dependencies(gazebo_rendering gazebo_msgs)
add_dependencies(gazebo_sensors gazebo_rendering)

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
  ${freeimage_LIBRARIES}
  ${TBB_LIBRARIES}
  ${ogre_libraries}
)

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

if (BUILD_OSX_BUNDLE)
  message ("bundle dirs ogre: ${OGRE_LIBRARY_DIRS}")
  message ("bundle dirs sformat: ${SDFormat_LIBRARY_DIRS}")

  install (TARGETS gzserver BUNDLE DESTINATION .)
  set (APPS "${CMAKE_BINARY_DIR}/gzserver.app/Contents/MacOS/gzserver")
  # Different libraries are not being installed into CMAKE_BINARY_DIR/lib
  # don't know why :/
  set (DIRS

      ${IGNITION-MATH_LIBRARY_DIRS}
      ${IGNITION-MSGS_LIBRARY_DIRS}
      ${IGNITION-TRANSPORT_LIBRARY_DIRS}
      ${SDFormat_LIBRARY_DIRS}
      ${OGRE_LIBRARY_DIRS}
      ${CMAKE_BINARY_DIR}/lib
      ${CMAKE_BINARY_DIR}/gazebo/util
      ${CMAKE_BINARY_DIR}/gazebo/math
      ${CMAKE_BINARY_DIR}/gazebo/msgs
      ${CMAKE_BINARY_DIR}/gazebo/physics
      ${CMAKE_BINARY_DIR}/gazebo/sensors
      ${CMAKE_BINARY_DIR}/gazebo/transport
      ${CMAKE_BINARY_DIR}/gazebo/common
      ${CMAKE_BINARY_DIR}/gazebo/rendering)

  SET(OGRE_COMPONENT_DIR gzserver.app/Contents/)
  FILE (GLOB OGRE_COMPONENTS "${OGRE_LIBRARY_DIRS}/libOgrePaging.*")
    INSTALL(FILES ${OGRE_COMPONENTS}
    DESTINATION ${OGRE_COMPONENT_DIR}/Components)

  INSTALL(CODE "
    file(GLOB_RECURSE LIBS
      \"\${CMAKE_INSTALL_PREFIX}/${OGRE_COMPONENT_DIR}/Components/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
    # Needed for getting the right permissions when using brew
    set (BU_CHMOD_BUNDLE_ITEMS ON)
    include(BundleUtilities)
    fixup_bundle(\"${APPS}\" \"\"  \"${DIRS}\")
  " COMPONENT Runtime)
endif()
