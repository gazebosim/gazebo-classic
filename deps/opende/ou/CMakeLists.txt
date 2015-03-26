add_definitions(-DUSE_PQP=0)
add_definitions(-DUSE_SVMLIGHT=0)

set (sources 
  src/atomic.cpp
  src/customization.cpp
  src/malloc.cpp
  src/threadlocalstorage.cpp
)

include_directories( 
  ${CMAKE_SOURCE_DIR}/deps/opende/ou/include
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -DdNODEBUG -DdDOUBLE -DHAVE_CONFIG_H -DPIC -D_OU_NAMESPACE=gazebo_odeou -DBUILDING_DLL_OU")

if (WIN32)
  add_library(gazebo_opende_ou SHARED ${sources})
else()
  gz_add_library(gazebo_opende_ou ${sources})
endif()
#target_link_libraries(gazebo_fcl gazebo_ccd gazebo_ann)
gz_install_library(gazebo_opende_ou)
