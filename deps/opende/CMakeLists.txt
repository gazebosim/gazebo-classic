include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (CheckIncludeFiles)
include (CheckFunctionExists)

CHECK_CXX_SOURCE_COMPILES("#include <alloca.h> 
int main() {alloca(0); return 1;}" HAVE_ALLOCA)


CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {atan2f(0,0); return 1;}" HAVE_ATAN2F)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {copysign(0,0); return 1;}" HAVE_COPYSIGN)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {copysignf(0,0); return 1;}" HAVE_COPYSIGNF)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {cosf(0); return 1;}" HAVE_COSF)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {fabsf(0); return 1;}" HAVE_FABSF)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {floor(0); return 1;}" HAVE_FLOOR)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {fmodf(0,0); return 1;}" HAVE_FMODF)

CHECK_CXX_SOURCE_COMPILES("#include <sys/time.h> 
int main() {gettimeofday(0,0); return 1;}" HAVE_GETTIMEOFDAY)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {isnan(0); return 1;}" HAVE_ISNAN)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {isnanf(0); return 1;}" HAVE_ISNANF)

CHECK_CXX_SOURCE_COMPILES("#include <stdlib.h> 
int main() {malloc(0); return 1;}" HAVE_MALLOC)

CHECK_CXX_SOURCE_COMPILES("#include <string.h> 
int main() {memmove(0,0,0); return 1;}" HAVE_MEMMOVE)

CHECK_CXX_SOURCE_COMPILES("#include <string.h> 
int main() {memset(0,0,0); return 1;}" HAVE_MEMSET)

CHECK_CXX_SOURCE_COMPILES("#include <stdlib.h> 
int main() {realloc(0,0); return 1;}" HAVE_REALLOC)

CHECK_CXX_SOURCE_COMPILES("#include <sys/select.h> 
int main() {select(0,0,0,0,0); return 1;}" HAVE_SELECT)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {sinf(0); return 1;}" HAVE_SINF)

CHECK_CXX_SOURCE_COMPILES("#include <stdio.h> 
int main() {sprintf(0,0); return 1;}" HAVE_SPRINTF)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {sqrt(0); return 1;}" HAVE_SQRT)

CHECK_CXX_SOURCE_COMPILES("#include <math.h> 
int main() {sqrtf(0); return 1;}" HAVE_SQRTF)

CHECK_CXX_SOURCE_COMPILES("#include <stdarg.h> 
#include <stdio.h>
int main() {va_list ap; vprintf(0,ap); return 1;}" HAVE_VPRINTF)

CHECK_CXX_SOURCE_COMPILES("#include <stdarg.h> 
#include <stdio.h>
int main() {va_list ap; vsnprintf(0,0,0,ap); return 1;}" HAVE_VSNPRINTF)


CHECK_INCLUDE_FILES(alloca.h HAVE_ALLOCA_H)
CHECK_INCLUDE_FILES(dlfcn.h HAVE_DLFCN_H)
CHECK_INCLUDE_FILES(glext.h HAVE_GL_GLEXT_H)
CHECK_INCLUDE_FILES(glu.h HAVE_GL_GLU_H)
CHECK_INCLUDE_FILES(gl.h HAVE_GL_GL_H)
CHECK_INCLUDE_FILES(inttypes.h HAVE_INTTYPES_H)
CHECK_INCLUDE_FILES(malloc.h HAVE_MALLOC_H)
CHECK_INCLUDE_FILES(math.h HAVE_MATH_H)
CHECK_INCLUDE_FILES(memory.h HAVE_MEMORY_H)
CHECK_INCLUDE_FILES(stdarg.h HAVE_STDARG_H)
CHECK_INCLUDE_FILES(stdbool.h HAVE_STDBOOL_H)
CHECK_INCLUDE_FILES(stdint.h HAVE_STDINT_H)
CHECK_INCLUDE_FILES(stdio.h HAVE_STDIO_H)
CHECK_INCLUDE_FILES(stdlib.h HAVE_STDLIB_H)
CHECK_INCLUDE_FILES(strings.h HAVE_STRINGS_H)
CHECK_INCLUDE_FILES(string.h HAVE_STRING_H)
CHECK_INCLUDE_FILES(sys/select.h HAVE_SYS_SELECT_H)
CHECK_INCLUDE_FILES(sys/socket.h HAVE_SYS_SOCKET_H)
CHECK_INCLUDE_FILES(sys/stat.h HAVE_SYS_STAT_H)
CHECK_INCLUDE_FILES(sys/time.h HAVE_SYS_TIME_H)
CHECK_INCLUDE_FILES(sys/types.h HAVE_SYS_TYPES_H)
CHECK_INCLUDE_FILES(time.h HAVE_TIME_H)
CHECK_INCLUDE_FILES(unistd.h HAVE_UNISTD_H)

configure_file(${CMAKE_SOURCE_DIR}/deps/opende/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h @ONLY)

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
