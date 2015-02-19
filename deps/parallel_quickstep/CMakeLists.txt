include (${gazebo_cmake_dir}/GazeboUtils.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(opende_PACKAGE_PATH ${CMAKE_SOURCE_DIR}/deps/opende)

##############################################################################


include_directories( 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${CMAKE_CURRENT_BINARY_DIR}/../opende
  ${CMAKE_SOURCE_DIR}/deps/opende/include
  ${CMAKE_SOURCE_DIR}/deps/opende/src
  ${CMAKE_SOURCE_DIR}/deps/parallel_quickstep/include/parallel_quickstep
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/deps/threadpool
)

link_directories(  
  ${Boost_LIBRARY_DIRS} 
)

set (NDEBUG bool true)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -DdNODEBUG -DdDOUBLE -DHAVE_CONFIG_H -DPIC")

if (SSE2_FOUND OR SSE3_FOUND OR SSSE3_FOUND OR SSE4_1_FOUND OR SSE4_2_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSSE")
endif()

set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}} -MF -MT -fno-strict-aliasing -DPIC ")

set(PARALLEL_QUICKSTEP_FLAGS -O3 )#-DTIMING)# -DVERBOSE -DBENCHMARKING -DERROR )
add_definitions(${PARALLEL_QUICKSTEP_FLAGS})

# default to CPU fall back so everyone can compile this package
set(USE_CPU "1")
#set(USE_CUDA "1")
#set(USE_OPENCL "1")
#set(USE_OPENMP "1")

################################################
# Automatically set USE_CUDA to 1 if it is found
# uncomment to enable
################################################
# CMake 2.8.0 or greater required for built-in CUDA module
#if( ${CMAKE_MINOR_VERSION} GREATER 6.2 )
#  if( ${CMAKE_MINOR_VERSION} LESS 8 )
#    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")
#  endif( ${CMAKE_MINOR_VERSION} LESS 8 )
#  find_package(CUDA QUIET)
#  if (CUDA_FOUND)
#    set(USE_CPU)
#    set(USE_CUDA "1")
#    MESSAGE(STATUS "CUDA Found, compiling with CUDA support")
#  else (CUDA_FOUND)
#    MESSAGE(STATUS "CUDA Not Found, using default CPU")
#  endif (CUDA_FOUND)
#endif( ${CMAKE_MINOR_VERSION} GREATER 6.2 )

if(DEFINED USE_CUDA)

  # CMake 2.8.0 or greater required for built-in CUDA module
  if( ${CMAKE_MINOR_VERSION} LESS 8 )
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")
  endif( ${CMAKE_MINOR_VERSION} LESS 8 )

  find_package(CUDA QUIET REQUIRED)

  include_directories( ${CUDA_INCLUDE_DIRS} )
  add_definitions(-DUSE_CUDA)

  set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)

  #default to oldest stuff
  set(CUDA_TARGET_SM "sm_10")

  if (NOT DEFINED CUDA_TARGET_SM)
    set(CUDA_TARGET_SM "sm_20")
  endif (NOT DEFINED CUDA_TARGET_SM)

  if (CUDA_TARGET_SM STREQUAL "sm_10")
    set(DOUBLE_SUPPORT_ENABLED FALSE)
    set(ATOMIC_SUPPORT_ENABLED FALSE)
    add_definitions(-DCUDA_SM10)
    message(STATUS "using sm_10 without DOUBLE_SUPPORT_ENABLED and ATOMIC_SUPPORT_ENABLED")
  elseif (CUDA_TARGET_SM STREQUAL "sm_11")
    set(DOUBLE_SUPPORT_ENABLED FALSE)
    set(ATOMIC_SUPPORT_ENABLED TRUE)
    add_definitions(-DCUDA_SM11)
    message(STATUS "using sm_11 with ATOMIC_SUPPORT_ENABLED")
  elseif (CUDA_TARGET_SM STREQUAL "sm_12")
    set(DOUBLE_SUPPORT_ENABLED FALSE)
    set(ATOMIC_SUPPORT_ENABLED TRUE)
    message(STATUS "using sm_12 with ATOMIC_SUPPORT_ENABLED")
    add_definitions(-DCUDA_SM12)
  elseif (CUDA_TARGET_SM STREQUAL "sm_13")
    set(DOUBLE_SUPPORT_ENABLED TRUE)
    set(ATOMIC_SUPPORT_ENABLED TRUE)
    message(STATUS "using sm_13 with DOUBLE_SUPPORT_ENABLED and ATOMIC_SUPPORT_ENABLED")
    add_definitions(-DCUDA_SM13)
  elseif (CUDA_TARGET_SM STREQUAL "sm_20")
    set(DOUBLE_SUPPORT_ENABLED TRUE)
    set(ATOMIC_SUPPORT_ENABLED TRUE)
    message(STATUS "using sm_20 with DOUBLE_SUPPORT_ENABLED and ATOMIC_SUPPORT_ENABLED")
    add_definitions(-DCUDA_SM20)
  else()
    message( FATAL_ERROR "Unknown CUDA_TARGET_SM: '${CUDA_TARGET_SM}', must be one of: 'sm_10' 'sm_11' 'sm_12' 'sm_13' 'sm_20'")
  endif()

  message(STATUS "CUDA Target Architecture: ${CUDA_TARGET_SM}")

  if (DOUBLE_SUPPORT_ENABLED)
    add_definitions(-DCUDA_DOUBLESUPPORT)
  else (DOUBLE_SUPPORT_ENABLED)
    add_definitions(-DCUDA_NODOUBLESUPPORT)
  endif (DOUBLE_SUPPORT_ENABLED)

  if (ATOMIC_SUPPORT_ENABLED)
    add_definitions(-DCUDA_ATOMICSUPPORT)
  else (ATOMIC_SUPPORT_ENABLED)
    add_definitions(-DCUDA_NOATOMICSUPPORT)
  endif (ATOMIC_SUPPORT_ENABLED)

  set(CUDA_SOURCE_FILES src/cuda_kernels.cu src/test_lib.cpp )
  set(CUDA_SOLVER_SOURCE_FILES
    src/parallel_stepper.cpp
    src/parallel_solver.cpp
    src/parallel_batch.cpp
    src/parallel_reduce.cpp
    src/parallel_quickstep.cpp
    src/cuda_solver.cpp)

  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -use_fast_math -O3 "-arch;${CUDA_TARGET_SM}" ${parallel_quickstep_CFLAGS_OTHER})

  cuda_compile(CUDA_GEN_FILES ${CUDA_SOURCE_FILES} SHARED -fPic)

  gz_add_library(gazebo_parallel_quickstep 
    ${CUDA_GEN_FILES}
    ${CUDA_SOURCE_FILES}
    ${CUDA_SOLVER_SOURCE_FILES}
    )
  add_executable(parallel_quickstep_lib_test src/main_for_lib.cpp )
  target_link_libraries(gazebo_parallel_quickstep gazebo_ode)
  target_link_libraries(gazebo_parallel_quickstep ${CUDA_LIBRARIES})
  target_link_libraries(gazebo_parallel_quickstep ${Boost_LIBRARIES})
  target_link_libraries(parallel_quickstep_lib_test gazebo_parallel_quickstep)
  cuda_build_clean_target()

elseif( DEFINED USE_OPENMP )

  add_definitions(-DUSE_OPENMP -fopenmp)

  set(OPENMP_SOLVER_SOURCE_FILES
    src/parallel_stepper.cpp
    src/parallel_solver.cpp
    src/parallel_batch.cpp
    src/parallel_reduce.cpp
    src/parallel_quickstep.cpp
    src/openmp_solver.cpp
    src/openmp_kernels.cpp )

  gz_add_library(gazebo_parallel_quickstep
    ${OPENMP_SOLVER_SOURCE_FILES}
    )
  target_link_libraries(gazebo_parallel_quickstep gazebo_ode)
  target_link_libraries(gazebo_parallel_quickstep ${Boost_LIBRARIES})
  set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp ")

elseif( DEFINED USE_OPENCL )

  set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")
  find_package( OpenCL REQUIRED )
  add_definitions(-DUSE_OPENCL -fopenmp)
  #include_directories( ${OPENCL_INCLUDE_DIRS} )
  include_directories( ${CUDA_TOOLKIT_ROOT_DIR}/include )

  set(OPENCL_SOLVER_SOURCE_FILES
    src/opencl_utils.cpp
    src/opencl_solver.cpp
    src/opencl_kernels.cpp
    src/parallel_stepper.cpp
    src/parallel_solver.cpp
    src/parallel_batch.cpp
    src/parallel_reduce.cpp
    src/parallel_quickstep.cpp)

  gz_add_library(gazebo_parallel_quickstep
    ${OPENCL_SOLVER_SOURCE_FILES}
    )

  target_link_libraries(gazebo_parallel_quickstep gazebo_ode)
  target_link_libraries(gazebo_parallel_quickstep ${OPENCL_LIBRARIES})
  target_link_libraries(gazebo_parallel_quickstep ${Boost_LIBRARIES})
  add_executable(parallel_quickstep_lib_test src/main_for_lib.cpp src/test_lib.cpp)
  target_link_libraries(parallel_quickstep_lib_test gazebo_parallel_quickstep)

elseif( DEFINED USE_CPU )

  add_definitions(-DUSE_CPU)

  set(CPU_SOLVER_SOURCE_FILES
    src/parallel_stepper.cpp
    src/parallel_quickstep.cpp)

  gz_add_library(gazebo_parallel_quickstep
    ${CPU_SOLVER_SOURCE_FILES}
    )

  target_link_libraries(gazebo_parallel_quickstep gazebo_ode)
  target_link_libraries(gazebo_parallel_quickstep ${Boost_LIBRARIES})
  set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp ")

endif()

set ( headers 
include/parallel_quickstep/cuda_common.h
include/parallel_quickstep/cuda_solver.h
include/parallel_quickstep/cuda_timer.h
include/parallel_quickstep/cuda_utils.h
include/parallel_quickstep/opencl_common.h
include/parallel_quickstep/opencl_solver.h
include/parallel_quickstep/opencl_utils.h
include/parallel_quickstep/openmp_common.h
include/parallel_quickstep/openmp_solver.h
include/parallel_quickstep/parallel_array.h
include/parallel_quickstep/parallel_batch.h
include/parallel_quickstep/parallel_common.h
include/parallel_quickstep/parallel_defines.h
include/parallel_quickstep/parallel_math.h
include/parallel_quickstep/parallel_memory.h
include/parallel_quickstep/parallel_quickstep.h
include/parallel_quickstep/parallel_reduce.h
include/parallel_quickstep/parallel_solver.h
include/parallel_quickstep/parallel_stepper.h
include/parallel_quickstep/parallel_timer.h
include/parallel_quickstep/parallel_utils.h
include/parallel_quickstep/vector_types.h
)

add_dependencies(gazebo_parallel_quickstep gazebo_ode)
gz_install_library(gazebo_parallel_quickstep)
gz_install_includes("parallel_quickstep" ${headers})
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fopenmp ")
