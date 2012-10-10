add_definitions(-DUSE_PQP=0)
add_definitions(-DUSE_SVMLIGHT=0)

set (sources 
  src/AABB.cpp
  src/OBB.cpp
  src/RSS.cpp
  src/vec_3f.cpp
  src/traversal_node_base.cpp
  src/traversal_node_bvhs.cpp
  src/intersect.cpp
  src/motion.cpp
  src/BV_fitter.cpp
  src/BV_splitter.cpp
  src/BVH_model.cpp
  src/BVH_utility.cpp
  src/transform.cpp
  src/simple_setup.cpp
  src/geometric_shapes.cpp
  src/geometric_shapes_utility.cpp
  src/geometric_shapes_intersect.cpp
  src/collision_node.cpp
  src/traversal_recurse.cpp
  src/broad_phase_collision.cpp
  src/collision.cpp
  src/collision_func_matrix.cpp
  src/interval_tree.cpp
  src/conservative_advancement.cpp
)

include_directories(SYSTEM
  ${CMAKE_SOURCE_DIR}/deps/fcl/include 
  ${CMAKE_SOURCE_DIR}/deps/ann/include 
  ${CMAKE_SOURCE_DIR}/deps/libccd/include 
  )

gz_add_library(gazebo_fcl ${sources})
target_link_libraries(gazebo_fcl gazebo_ccd gazebo_ann)
gz_install_library(gazebo_fcl)
