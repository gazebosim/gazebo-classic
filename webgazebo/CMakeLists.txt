SET (sources WebGazebo.cc 
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
  ${WEBSIM_LINK_DIRS} 
  ${boost_library_dirs} 
  ${CMAKE_BINARY_DIR}/libgazebo 
)

SET_SOURCE_FILES_PROPERTIES(${sources} PROPERTIES COMPILE_FLAGS 
                            '-DINSTALL_PREFIX="/usr/local/"')
ADD_LIBRARY(webgz ${sources})

ADD_EXECUTABLE(webgazebo main.cc)
ADD_EXECUTABLE(client client.cc)

TARGET_LINK_LIBRARIES( webgz yaml event ${WEBSIM_LINK_LIBS})
TARGET_LINK_LIBRARIES( webgazebo gazebo yaml event ${WEBSIM_LINK_LIBS} ${boost_libraries} webgz)
TARGET_LINK_LIBRARIES( client gazebo yaml event ${WEBSIM_LINK_LIBS} ${boost_libraries} webgz)

INSTALL(TARGETS webgazebo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
INSTALL(TARGETS webgz DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
