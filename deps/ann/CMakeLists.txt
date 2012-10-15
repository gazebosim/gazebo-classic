add_definitions(-DUSE_PQP=0)
add_definitions(-DUSE_SVMLIGHT=0)

set (sources 
src/ANN.cpp
src/bd_fix_rad_search.cpp
src/bd_pr_search.cpp
src/bd_search.cpp
src/bd_tree.cpp
src/bd_tree.h
src/brute.cpp
src/kd_dump.cpp
src/kd_fix_rad_search.cpp
src/kd_fix_rad_search.h
src/kd_pr_search.cpp
src/kd_pr_search.h
src/kd_search.cpp
src/kd_search.h
src/kd_split.cpp
src/kd_split.h
src/kd_tree.cpp
src/kd_tree.h
src/kd_util.cpp
src/kd_util.h
src/perf.cpp
src/pr_queue.h
src/pr_queue_k.h
)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/ann/include )

gz_add_library(gazebo_ann ${sources})
gz_install_library(gazebo_ann)
