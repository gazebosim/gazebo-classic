include_directories (
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
  ${assimp_library_dirs} 
  ${ODE_LIBRARY_DIRS}
)


enable_testing()
find_package( GTest REQUIRED )
include_directories( ${GTEST_INCLUDE_DIRS} )
add_subdirectory( benchmarks )

#add_executable(factory factory.cc ${PROJECT_SOURCE_DIR}/src/Server.cc ${PROJECT_SOURCE_DIR}/src/Master.cc)

#target_link_libraries( factory ${GTEST_BOTH_LIBRARIES} 
#                               gazebo_common 
#                               gazebo_transport 
#                               gazebo_physics
#                               gazebo_sensors
#                               gazebo_rendering
#                               gazebo_msgs
#                               assimp
#                               pthread 
#                               )
#
#add_test(FactoryTest factory)
