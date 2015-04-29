add_definitions(-DUSE_PQP=0)
add_definitions(-DUSE_SVMLIGHT=0)

set (sources 
  src/alloc.c
  src/ccd.c
  src/mpr.c
  src/polytope.c
  src/support.c
  src/vec3.c
  )

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/libccd/include )


gz_add_library(gazebo_ccd ${sources})
target_link_libraries(gazebo_ccd m)
gz_install_library(gazebo_ccd)
