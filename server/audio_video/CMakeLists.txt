SET (sources AudioDecoder.cc
             OpenAL.cc
)

SET (headers AudioDecoder.hh
             OpenAL.hh
)

#APPEND_TO_SERVER_SOURCES(${sources})
#APPEND_TO_SERVER_HEADERS(${headers})

ADD_LIBRARY(gazebo_av STATIC ${sources})
ADD_LIBRARY(gazebo_av-shared SHARED ${sources})

SET_TARGET_PROPERTIES(gazebo_av-shared PROPERTIES OUTPUT_NAME "gazebo_av")
INSTALL (TARGETS gazebo_av gazebo_av-shared DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
