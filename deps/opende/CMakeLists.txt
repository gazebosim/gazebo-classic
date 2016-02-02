include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (CheckIncludeFiles)
include (CheckFunctionExists)
include (CheckLibraryExists)

include_directories(SYSTEM
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${CMAKE_SOURCE_DIR}/deps/opende/include
  ${CMAKE_SOURCE_DIR}/deps/opende/ou/include
  ${CMAKE_SOURCE_DIR}/deps/opende/OPCODE
  ${CMAKE_SOURCE_DIR}/deps/opende/GIMPACT/include
  ${CMAKE_SOURCE_DIR}/deps/threadpool
  ${Boost_INCLUDE_DIRS}
  ${CCD_INCLUDE_DIRS}
)

link_directories(  
  ${CCD_LIBRARY_DIRS} 
  ${Boost_LIBRARY_DIR}
)

if (HAVE_DART)
  include_directories(${DARTCore_INCLUDE_DIRS})
  link_directories(${DARTCore_LIBRARY_DIRS})
endif()

if (HAVE_BULLET)
  include_directories(${BULLET_INCLUDE_DIRS})
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

add_subdirectory(OPCODE)
add_subdirectory(GIMPACT)
add_subdirectory(ou)

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

CHECK_CXX_SOURCE_COMPILES("#include <stdio.h> 
int main() {snprintf(0,0,0); return 1;}" HAVE_SNPRINTF)

CHECK_CXX_SOURCE_COMPILES("#include <limits> 
int main() {return 1;}" HAVE_LIMITS)

CHECK_CXX_SOURCE_COMPILES("#include <boost/shared_array.hpp> 
int main() {return 1;}" HAVE_BOOST_SHARED_ARRAY_HPP)

set (NDEBUG bool true)

CHECK_INCLUDE_FILES(alloca.h HAVE_ALLOCA_H)
CHECK_INCLUDE_FILES(dlfcn.h HAVE_DLFCN_H)
CHECK_INCLUDE_FILES(float.h HAVE_FLOAT_H)
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
CHECK_INCLUDE_FILES(obstack.h HAVE_OBSTACK)

CHECK_LIBRARY_EXISTS(m cos "" HAVE_LIBM)

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
src/heightfield.cpp
src/io.cpp
src/ioh5.cpp
src/lcp.cpp
src/mass.cpp
src/mat.cpp
src/matrix.cpp
src/memory.cpp
src/misc.cpp
src/obstack.cpp
src/ode.cpp
src/odeinit.cpp
src/odemath.cpp
src/odeou.cpp
src/odetls.cpp
src/optimize.cpp
src/plane.cpp
src/quickstep.cpp
src/quickstep_cg_lcp.cpp
src/quickstep_pgs_lcp.cpp
src/quickstep_update_bodies.cpp
src/quickstep_util.cpp
src/ray.cpp
src/robuststep.cpp
src/rotation.cpp
src/sphere.cpp
src/step.cpp
src/step_bullet_lemke_wrapper.cpp
src/step_bullet_pgs_wrapper.cpp
src/step_dart_pgs_wrapper.cpp
src/symm.c
src/timer.cpp
src/util.cpp
src/fastdot.c
src/fastldlt.c
src/fastlsolve.c
src/fastltsolve.c
src/nextafterf.c
src/joints/amotor.cpp
src/joints/ball.cpp
src/joints/contact.cpp
src/joints/fixed.cpp
src/joints/hinge2.cpp
src/joints/hinge.cpp
src/joints/joint.cpp
src/joints/joints.h
src/joints/lmotor.cpp
src/joints/null.cpp
src/joints/piston.cpp
src/joints/plane2d.cpp
src/joints/pr.cpp
src/joints/pu.cpp
src/joints/screw.cpp
src/joints/dhinge.cpp
src/joints/dball.cpp
src/joints/gearbox.cpp
src/joints/slider.cpp
src/joints/universal.cpp
)

set ( headers 
include/ode/collision.h
include/ode/collision_space.h
include/ode/collision_trimesh.h
include/ode/common.h
include/ode/compatibility.h
include/ode/contact.h
include/ode/error.h
include/ode/export-dif.h
include/ode/h5dump.h
include/ode/mass.h
include/ode/matrix.h
include/ode/memory.h
include/ode/misc.h
include/ode/objects.h
include/ode/odeconfig.h
include/ode/odecpp_collision.h
include/ode/odecpp.h
include/ode/ode.h
include/ode/odeinit.h
include/ode/odemath.h
include/ode/odemath_legacy.h
include/ode/rotation.h
include/ode/timer.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -DdNODEBUG -DdDOUBLE -DHAVE_CONFIG_H -DPIC")

if (SSE2_FOUND OR SSE3_FOUND OR SSSE3_FOUND OR SSE4_1_FOUND OR SSE4_2_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DODE_SSE")
endif()

if (WIN32)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWIN32 -DODE_DLL")
endif()

set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}} -MF -MT -fno-strict-aliasing -DPIC ")

if (WIN32)
  add_library(gazebo_ode SHARED ${sources})
else()
  gz_add_library(gazebo_ode ${sources})
endif()

target_link_libraries(gazebo_ode
  gazebo_opcode
  gazebo_gimpact
  gazebo_opende_ou
  ${CCD_LIBRARIES}
  ${Boost_LIBRARIES})

if (HDF5_FOUND AND HDF5_INSTRUMENT)
  message(STATUS "HDF5 Found and Instrument enabled")
  target_link_libraries(gazebo_ode ${HDF5_LIBRARIES})
endif()

add_dependencies(gazebo_ode gazebo_opcode gazebo_gimpact)
add_dependencies(gazebo_ode gazebo_opende_ou)

gz_install_library(gazebo_ode)
gz_install_includes("ode" ${headers})
