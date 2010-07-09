INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)
STRING (REPLACE " " ";" boost_include_dirs_split "${boost_include_dirs}")
STRING (REPLACE " " ";" boost_library_dirs_split "${boost_library_dirs}")

INCLUDE_DIRECTORIES(
  .
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${boost_include_dirs_split}
)

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${boost_library_dirs_split} 
)


set (sources gazebomodel.cc gazebobody.cc gazeboplugin.cc Toolbase.cc)

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-ggdb -g2 -Wall")

add_executable(gazebomodel gazebomodel.cc Toolbase.cc)
add_executable(gazebobody gazebobody.cc Toolbase.cc)
add_executable(gazeboplugin gazeboplugin.cc Toolbase.cc)

target_link_libraries(gazebomodel gazeboshm yaml ${boost_libraries})
target_link_libraries(gazebobody gazeboshm yaml ${boost_libraries})
target_link_libraries(gazeboplugin gazeboshm yaml ${boost_libraries})

set_target_properties(gazebomodel PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(gazebomodel PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(gazebobody PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(gazebobody PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(gazeboplugin PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(gazeboplugin PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")


install (TARGETS gazebomodel gazebobody gazeboplugin DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
