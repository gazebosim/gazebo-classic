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
include_directories(${GTEST_INCLUDE_DIRS})

set (gtest_sources
  common.cc
  factory.cc
  file_handling.cc
  heightmap.cc
  laser.cc
  math.cc
  msgs.cc
  physics.cc
  pioneer2dx.cc
  pr2.cc
  projector.cc
  speed.cc
  transport.cc)

set (plugins )

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

# Build plugins
foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
endforeach (src ${plugins})

# Build tests
foreach(GTEST_SOURCE_file ${gtest_sources})
  string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
  add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${PROJECT_SOURCE_DIR}/src/Server.cc ${PROJECT_SOURCE_DIR}/src/Master.cc ${PROJECT_SOURCE_DIR}/src/gazebo.cc)

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
  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
  set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 60)

endforeach()
