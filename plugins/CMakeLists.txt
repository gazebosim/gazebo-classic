project(benchmarks)

cmake_minimum_required( VERSION 2.4 FATAL_ERROR )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0004 NEW)
endif(COMMAND cmake_policy)

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazeboserver)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})
endif (PKG_CONFIG_FOUND)

set (plugins pioneer_line 
             pioneer_circle
             pioneer_gripper
             ball_drop
             box_push
             box_stack
             )


foreach (src ${plugins})
  set_source_files_properties(${src}.cc PROPERTIES COMPILE_FLAGS ${GAZEBO_CFLAGS})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} gazebo ${GAZEBO_LINK_LIBS} )
  set_target_properties(${src} PROPERTIES SKIP_BUILD_RPATH TRUE)
  set_target_properties(${src} PROPERTIES LINK_FLAGS "${GAZEBO_LDFLAGS}")
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/plugins/)
endforeach (src ${benchmarks})
