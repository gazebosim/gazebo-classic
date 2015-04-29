include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${CMAKE_SOURCE_DIR}/deps/opende/GIMPACT/include
)


########################################
# Include all the search paths for headers

set ( sources 
  src/gim_tri_tri_overlap.cpp
  src/gim_math.cpp
  src/gimpact.cpp
  src/gim_trimesh_trimesh_collision.cpp
  src/gim_trimesh_sphere_collision.cpp
  src/gim_trimesh.cpp
  src/gim_trimesh_capsule_collision.cpp
  src/gim_memory.cpp
  src/gim_contact.cpp
  src/gim_boxpruning.cpp
  src/gim_trimesh_ray_collision.cpp
)

set ( headers 
  include/GIMPACT/gim_tri_sphere_collision.h
  include/GIMPACT/gim_tri_capsule_collision.h
  include/GIMPACT/gimpact.h
  include/GIMPACT/gim_trimesh.h
  include/GIMPACT/gim_tri_collision.h
  include/GIMPACT/gim_radixsort.h
  include/GIMPACT/gim_memory.h
  include/GIMPACT/gim_geometry.h
  include/GIMPACT/gim_contact.h
  include/GIMPACT/gim_boxpruning.h
  include/GIMPACT/gim_math.h
)

gz_add_library(gazebo_gimpact ${sources})
gz_install_library(gazebo_gimpact)
gz_install_includes("GIMPACT" ${headers})

