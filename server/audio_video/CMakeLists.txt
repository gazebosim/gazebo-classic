INCLUDE_DIRECTORIES( 
  ${CMAKE_SOURCE_DIR}/server
  ${LIBAVCODEC_PATH}
  ${LIBAVFORMAT_PATH}
  ${gazeboserver_include_dirs}
)

SET (sources AudioDecoder.cc
             OpenAL.cc
)

ADD_LIBRARY(gazebo_av STATIC ${sources})
