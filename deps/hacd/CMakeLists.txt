#include(FindOpenMP)
#if(OPENMP_FOUND)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

set (sources
src/hacdGraph.cpp
src/hacdHACD.cpp
src/hacdICHull.cpp
src/hacdManifoldMesh.cpp
src/hacdMeshDecimator.cpp
src/hacdMicroAllocator.cpp
src/hacdRaycastMesh.cpp
)

include_directories(SYSTEM
  ${CMAKE_SOURCE_DIR}/deps/hacd/include
)

gz_add_library(gazebo_hacd ${sources})
gz_install_library(gazebo_hacd)
