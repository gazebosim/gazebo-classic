SET (sources   WebGazebo.cc  
               Properties.cc
             ../server/Quatern.cc 
             ../server/Vector3.cc 
             ../server/Angle.cc
)

INCLUDE_DIRECTORIES( 
  ${CMAKE_SOURCE_DIR}/server
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${WEBSIM_INCLUDE_DIRS} 
  ${boost_include_dirs}
)

LINK_DIRECTORIES( 
  ${WEBSIM_LIBRARY_DIRS} 
  ${boost_library_dirs} 
  ${CMAKE_BINARY_DIR}/libgazebo 
)

SET_SOURCE_FILES_PROPERTIES(${sources} PROPERTIES COMPILE_FLAGS 
                            '-DINSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}"')

ADD_LIBRARY(webgz ${sources})

ADD_EXECUTABLE(webgazebo main.cc)
ADD_EXECUTABLE(client client.cc)

TARGET_LINK_LIBRARIES( webgz event ${WEBSIM_LINK_LIBS})
TARGET_LINK_LIBRARIES( webgazebo gazebo event ${WEBSIM_LINK_LIBS} ${boost_libraries} webgz)
TARGET_LINK_LIBRARIES( client gazebo event ${WEBSIM_LINK_LIBS} ${boost_libraries} webgz)

INSTALL(TARGETS webgazebo webgz client 
       LIBRARY DESTINATION lib
       ARCHIVE DESTINATION lib
       RUNTIME DESTINATION bin )
