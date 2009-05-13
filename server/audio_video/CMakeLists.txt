INCLUDE_DIRECTORIES( 
  ${CMAKE_SOURCE_DIR}/server
  ${LIBAVCODEC_PATH}
  ${LIBAVFORMAT_PATH}
  ${gazeboserver_include_dirs}
  ${boost_include_dirs}
)

SET (sources AudioDecoder.cc
             OpenAL.cc
)

SET (headers AudioDecoder.hh
             OpenAL.hh
)

APPEND_TO_SERVER_HEADERS(${headers})

ADD_LIBRARY(gazebo_av STATIC ${sources})
