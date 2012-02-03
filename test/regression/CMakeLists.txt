include_directories (
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
  ${ODE_LIBRARY_DIRS}
)

if (INCLUDE_PLAYER)
  add_subdirectory(player)
endif()

find_package(GTest REQUIRED)
include_directories( ${GTEST_INCLUDE_DIRS} )

set (gtest_sources transport.cc speed.cc factory.cc physics.cc msgs.cc math.cc pr2.cc)

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
    )
  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})

  #file(STRINGS ${GTEST_SOURCE_file} GTEST_NAMES REGEX ^TEST)
    #foreach(GTEST_NAME ${GTEST_NAMES})
      #  string(REGEX REPLACE ["\) \(,"] ";" GTEST_NAME ${GTEST_NAME})
      #  list(GET GTEST_NAME 1 GTEST_GROUP_NAME)
      #  list(GET GTEST_NAME 3 GTEST_NAME)
      #  add_test(${GTEST_GROUP_NAME}.${GTEST_NAME}
      #           ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
      #           --gtest_filter=${GTEST_GROUP_NAME}.${GTEST_NAME})
  #endforeach()
endforeach()
