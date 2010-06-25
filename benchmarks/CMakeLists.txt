project(benchmarks)

cmake_minimum_required( VERSION 2.4 FATAL_ERROR )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0004 NEW)
endif(COMMAND cmake_policy)


include (${gazebo_cmake_dir}/GazeboUtils.cmake)
string (REPLACE " " ";" boost_include_dirs_split "${boost_include_dirs}")
string (REPLACE " " ";" boost_library_dirs_split "${boost_library_dirs}")

include_directories(
 .
  ${CMAKE_SOURCE_DIR}/libgazebo
  ${LINK_DIRS}
  ${boost_include_dirs_split}
)

link_directories(
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${INCLUDE_DIRS}
)

set (benchmarks laser 
                box_grid 
                box_pile 
                joint_chain 
                camera 
                spinningbox 
                pendulum 
                box_drop 
                table
                pioneer_circle
                pioneer_spiral
                )

foreach (src ${benchmarks})
  set_source_files_properties(${src}_benchmark.cc PROPERTIES COMPILE_FLAGS "-Wall")
  add_executable(${src}_benchmark ${src}_benchmark.cc)
  target_link_libraries(${src}_benchmark  gazebo ${LINK_LIBS} )
  set_target_properties(${src}_benchmark  PROPERTIES SKIP_BUILD_RPATH TRUE)
  set_target_properties(${src}_benchmark  PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
  install (TARGETS ${src}_benchmark DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endforeach (src ${benchmarks})
