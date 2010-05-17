project(benchmarks)

cmake_minimum_required( VERSION 2.4 FATAL_ERROR )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0004 NEW)
endif(COMMAND cmake_policy)

include (FindPkgConfig)

# if (PKG_CONFIG_FOUND)
#   pkg_check_modules(PKG_CFGS REQUIRED libgazebo)
#   set (INCLUDE_DIRS ${PKG_CFGS_INCLUDE_DIRS})
#   set (LINK_DIRS  ${PKG_CFGS_LIBRARY_DIRS})
#   set (LINK_LIBS ${PKG_CFGS_LINK_LIBS} ${PKG_CFGS_LIBRARIES})
# endif (PKG_CONFIG_FOUND)

link_directories(../libgazebo ${INCLUDE_DIRS})
include_directories(../libgazebo ${LINK_DIRS})

include_directories(.)

set (sources laser_benchmark.cc 
             box_grid_benchmark.cc 
             box_pile_benchmark.cc 
             joint_chain_benchmark.cc
             camera_benchmark.cc
             spinningbox_benchmark.cc
             pendulum_benchmark.cc
             )

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-ggdb -g2 -Wall")

add_executable(laser_benchmark laser_benchmark.cc)
add_executable(box_pile_benchmark box_pile_benchmark.cc)
add_executable(box_grid_benchmark box_grid_benchmark.cc)
add_executable(joint_chain_benchmark joint_chain_benchmark.cc)
add_executable(camera_benchmark camera_benchmark.cc)
add_executable(spinningbox_benchmark spinningbox_benchmark.cc)
add_executable(pendulum_benchmark pendulum_benchmark.cc)

target_link_libraries(laser_benchmark       gazebo ${LINK_LIBS} )
target_link_libraries(box_grid_benchmark    gazebo ${LINK_LIBS} )
target_link_libraries(box_pile_benchmark    gazebo ${LINK_LIBS} )
target_link_libraries(joint_chain_benchmark gazebo ${LINK_LIBS} )
target_link_libraries(camera_benchmark      gazebo ${LINK_LIBS} )
target_link_libraries(spinningbox_benchmark gazebo ${LINK_LIBS} )
target_link_libraries(pendulum_benchmark    gazebo ${LINK_LIBS} )

set_target_properties(laser_benchmark       PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(box_grid_benchmark    PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(box_pile_benchmark    PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(joint_chain_benchmark PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(camera_benchmark      PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(spinningbox_benchmark   PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(pendulum_benchmark      PROPERTIES SKIP_BUILD_RPATH TRUE)

set_target_properties(laser_benchmark       PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(box_grid_benchmark    PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(box_pile_benchmark    PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(joint_chain_benchmark PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(camera_benchmark      PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(spinningbox_benchmark PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(pendulum_benchmark    PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")

install (TARGETS laser_benchmark box_grid_benchmark box_pile_benchmark joint_chain_benchmark camera_benchmark spinningbox_benchmark pendulum_benchmark DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
