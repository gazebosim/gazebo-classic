include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (CheckIncludeFiles)


CHECK_INCLUDE_FILES( )


########################################
# Include all the search paths for headers

set ( sources 
src/array.cpp
src/box.cpp
src/capsule.cpp
src/collision_cylinder_box.cpp
src/collision_cylinder_plane.cpp
src/collision_cylinder_sphere.cpp
src/collision_cylinder_trimesh.cpp
src/collision_kernel.cpp
src/collision_libccd.cpp
src/collision_quadtreespace.cpp
src/collision_sapspace.cpp
src/collision_space.cpp
src/collision_transform.cpp
src/collision_trimesh_box.cpp
src/collision_trimesh_ccylinder.cpp
src/collision_trimesh_disabled.cpp
src/collision_trimesh_distance.cpp
src/collision_trimesh_gimpact.cpp
src/collision_trimesh_opcode.cpp
src/collision_trimesh_plane.cpp
src/collision_trimesh_ray.cpp
src/collision_trimesh_sphere.cpp
src/collision_trimesh_trimesh.cpp
src/collision_trimesh_trimesh_new.cpp
src/collision_util.cpp
src/convex.cpp
src/cylinder.cpp
src/error.cpp
src/export-dif.cpp
src/fastdot.c
src/fastldlt.c
src/fastlsolve.c
src/fastltsolve.c
src/heightfield.cpp
src/lcp.cpp
src/mass.cpp
src/mat.cpp
src/matrix.cpp
src/memory.cpp
src/misc.cpp
src/nextafterf.c
src/obstack.cpp
src/ode.cpp
src/odeinit.cpp
src/odemath.cpp
src/odeou.cpp
src/odetls.cpp
src/plane.cpp
src/quickstep.cpp
src/ray.cpp
src/rotation.cpp
src/sphere.cpp
src/step.cpp
src/timer.cpp
src/util.cpp
src/joints/amotor.cpp
src/joints/ball.cpp
src/joints/contact.cpp
src/joints/fixed.cpp
src/joints/hinge2.cpp
src/joints/hinge.cpp
src/joints/joint.cpp
src/joints/lmotor.cpp
src/joints/null.cpp
src/joints/piston.cpp
src/joints/plane2d.cpp
src/joints/pr.cpp
src/joints/pu.cpp
src/joints/slider.cpp
src/joints/universal.cpp
)

set ( headers 
src/collision_space_internal.h
src/collision_std.h
src/collision_transform.h
src/collision_trimesh_colliders.h
src/collision_libccd.h
src/collision_kernel.h
src/collision_trimesh_internal.h
src/collision_util.h
src/heightfield.h
src/lcp.h
src/mat.h
src/objects.h
src/obstack.h
src/odeou.h
src/odetls.h
src/quickstep.h
src/step.h
src/util.h
src/joints/amotor.h
src/joints/ball.h
src/joints/contact.h
src/joints/fixed.h
src/joints/hinge2.h
src/joints/hinge.h
src/joints/joint_internal.h
src/joints/joint.h
src/joints/joints.h
src/joints/lmotor.h
src/joints/null.h
src/joints/plane2d.h
src/joints/piston.h
src/joints/pr.h
src/joints/slider.h
src/joints/universal.h
src/joints/pu.h
src/array.h
include/collision.h
include/collision_space.h
include/collision_trimesh.h
include/common.h
include/compatibility.h
include/contact.h
include/error.h
include/export-dif.h
include/Makefile.am
include/mass.h
include/matrix.h
include/memory.h
include/misc.h
include/objects.h
include/odeconfig.h
include/odecpp_collision.h
include/odecpp.h
include/ode.h
include/odeinit.h
include/odemath.h
include/odemath_legacy.h
include/rotation.h
include/timer.h
)

add_library(gazebo_ode SHARED ${sources})
#target_link_libraries( gazebo_ode )

INSTALL_LIBRARY(gazebo_ode)
INSTALL_INCLUDES("ode" ${headers})
