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
  ${ODE_LIBRARY_DIRS}
)

if (GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_subdirectory(regression)
endif()
