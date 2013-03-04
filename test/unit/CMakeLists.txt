include_directories (
  ${PROJECT_SOURCE_DIR}/gazebo 
  ${PROJECT_BINARY_DIR}/gazebo
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${Boost_LIBRARY_DIRS} 
  ${ODE_LIBRARY_DIRS}
)

# Disable the player test for now, until we figure out a better way to test them
#if (INCLUDE_PLAYER)
#  add_subdirectory(player)
#endif()

include_directories(${PROJECT_SOURCE_DIR}/test/gtest/include)

set (gtest_sources
  common.cc
  logwrite.cc
)

# Build plugins
foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
endforeach (src ${plugins})

# Build tests
foreach(GTEST_SOURCE_file ${gtest_sources})
  string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
  add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${PROJECT_SOURCE_DIR}/gazebo/Server.cc ${PROJECT_SOURCE_DIR}/gazebo/Master.cc ${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc)

target_link_libraries(${BINARY_NAME}
  ${GTEST_LIBRARY} 
  ${GTEST_MAIN_LIBRARY} 
    gazebo_common
    gazebo_sdf_interface
    gazebo_transport
    gazebo_physics
    gazebo_physics_rtql8
    gazebo_sensors
    gazebo_rendering
    gazebo_msgs
    pthread
    ${tinyxml_libraries}
    )
  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
  set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)
  # Check that the test produced a result and create a failure if it didn't.
  # Guards against crashed and timed out tests.
  add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
endforeach()
