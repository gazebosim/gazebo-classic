include_directories (
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${PLAYER_INCLUDE_DIRS} 
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
  ${ODE_LIBRARY_DIRS}
  ${PLAYER_LINK_DIRS}
)

find_package(GTest REQUIRED)
include_directories( ${GTEST_INCLUDE_DIRS} )

set (gtest_sources player.cc)

foreach(GTEST_SOURCE_file ${gtest_sources})
  string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
  add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${PROJECT_SOURCE_DIR}/src/Server.cc ${PROJECT_SOURCE_DIR}/src/Master.cc)

  target_link_libraries( ${BINARY_NAME} ${GTEST_BOTH_LIBRARIES} 
    gazebo_common
    gazebo_sdf_interface
    gazebo_transport
    gazebo_physics
    gazebo_sensors
    gazebo_rendering
    gazebo_msgs
    pthread
    ${tinyxml_libraries}
    ${PLAYER_LINK_LIBS}
    )
  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})

endforeach()
