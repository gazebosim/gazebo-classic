include_directories (
  ${PROJECT_SOURCE_DIR}/gazebo 
  ${PROJECT_BINARY_DIR}/gazebo 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${Boost_LIBRARY_DIRS} 
  ${ODE_LIBRARY_DIRS}
)

if (GTEST_FOUND)
  execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
  execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_subdirectory(regression)
endif()
