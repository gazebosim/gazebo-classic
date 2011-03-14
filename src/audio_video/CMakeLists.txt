include_directories( ${OAL_INCLUDE_DIRS} )

link_directories( ${OAL_LIBRARY_DIRS} )

set (sources AudioDecoder.cc
             OpenAL.cc
)

set (headers AudioDecoder.hh
             OpenAL.hh
)

add_library(gazebo_av SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_av-shared PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries(gazebo_av ${OAL_LIBRARIES} ${AVC_LIBRARIES} ${AVF_LIBRARIES} ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})

set_target_properties(gazebo_av PROPERTIES OUTPUT_NAME "gazebo_av" VERSION ${GAZEBO_VERSION})

install (TARGETS gazebo_av DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
