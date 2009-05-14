SET (sources AudioDecoder.cc
             OpenAL.cc
)

SET (headers AudioDecoder.hh
             OpenAL.hh
)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})

#ADD_LIBRARY(gazebo_av STATIC ${sources})
