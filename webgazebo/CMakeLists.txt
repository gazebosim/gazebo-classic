SET (sources WebGazebo.cc ../server/Quatern.cc ../server/Vector3.cc ../server/Angle.cc)

INCLUDE_DIRECTORIES( .. ../server ${WEBSIM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/libgazebo )
LINK_DIRECTORIES( ${WEBSIM_LINK_DIRS} gazeboshm)

SET_SOURCE_FILES_PROPERTIES(${sources} PROPERTIES COMPILE_FLAGS 
                            '-DINSTALL_PREFIX="/usr/local/"')
ADD_LIBRARY(webgz ${sources})

ADD_EXECUTABLE(webgazebo main.cc)
ADD_EXECUTABLE(client client.cc)

TARGET_LINK_LIBRARIES( webgz yaml event ${WEBSIM_LINK_LIBS})
TARGET_LINK_LIBRARIES( webgazebo gazeboshm yaml event ${WEBSIM_LINK_LIBS} ${Boost_LIBRARIES} webgz)
TARGET_LINK_LIBRARIES( client gazeboshm yaml event ${WEBSIM_LINK_LIBS} ${Boost_LIBRARIES} webgz)

INSTALL(TARGETS webgazebo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
INSTALL(TARGETS webgz DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
