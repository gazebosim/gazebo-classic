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

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_CCD")

if (WIN32)
  add_library(gazebo_ccd SHARED ${sources})
else()
  gz_add_library(gazebo_ccd ${sources})
endif()

if (NOT WIN32)
  target_link_libraries(gazebo_ccd m)
endif()
gz_install_library(gazebo_ccd)
