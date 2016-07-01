include_directories (
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PLAYER_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(
  ${OGRE_LIBRARY_PATH}
  ${Boost_LIBRARY_DIRS}
  ${ODE_LIBRARY_DIRS}
  ${PLAYER_LINK_DIRS}
)

find_package(GTest REQUIRED)
include_directories( ${GTEST_INCLUDE_DIRS} )

set (gtest_sources player.cc)

foreach(GTEST_SOURCE_file ${gtest_sources})
  string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
  add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${PROJECT_SOURCE_DIR}/gazebo/Server.cc ${PROJECT_SOURCE_DIR}/gazebo/Master.cc ${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc)

  target_link_libraries(${BINARY_NAME} ${GTEST_BOTH_LIBRARIES}
    gazebo_common
    gazebo_transport
    gazebo_physics
    gazebo_sensors
    gazebo_rendering
    gazebo_msgs
    ${PLAYER_LINK_LIBS}
    )

  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})
endforeach()
