INCLUDE_DIRECTORIES(
  ${OAL_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${OAL_LIBRARY_DIRS}
)

SET (sources AudioDecoder.cc
             OpenAL.cc
)

SET (headers AudioDecoder.hh
             OpenAL.hh
)

#ADD_LIBRARY(gazebo_av STATIC ${sources})
ADD_LIBRARY(gazebo_av-shared SHARED ${sources})


if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_av-shared PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})


#TARGET_LINK_LIBRARIES(gazebo_av ${OAL_LIBRARIES} ${AVC_LIBRARIES} ${AVF_LIBRARIES})
TARGET_LINK_LIBRARIES(gazebo_av-shared ${OAL_LIBRARIES} ${AVC_LIBRARIES} ${AVF_LIBRARIES})

#SET_TARGET_PROPERTIES(gazebo_av PROPERTIES OUTPUT_NAME "gazebo_av")
SET_TARGET_PROPERTIES(gazebo_av-shared PROPERTIES OUTPUT_NAME "gazebo_av" VERSION ${GAZEBO_VERSION})

INSTALL (TARGETS gazebo_av-shared DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
