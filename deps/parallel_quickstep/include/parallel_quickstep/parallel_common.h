#ifndef PARALLEL_COMMON_H
#define PARALLEL_COMMON_H

#include <ode/ode.h>
#include <stdlib.h>
#include <vector>

namespace ParallelTypes
{
  enum ParallelType { CUDA, OpenCL, OpenMP, None };
}
typedef ParallelTypes::ParallelType ParallelType;

#if defined(USE_CUDA)

#include <cuda_runtime.h>
#include <math_constants.h>
#include "cuda_common.h"
#define PARALLEL_TYPE CUDA
#define PARALLEL_ENABLED 1

#elif defined(USE_OPENCL)

#include <CL/cl.h>
#include <CL/cl_ext.h>
#include <vector_types.h>
#include <opencl_utils.h>
#include <opencl_common.h>
#define PARALLEL_TYPE OpenCL
#define PARALLEL_ENABLED 1

#ifdef dSINGLE
#define KERNEL_PREFIX "#define make_float4(a) (float4)(a) \n\n"
#define KERNEL_OPTIONS "-D dSINGLE -cl-fast-relaxed-math"
#elif dDOUBLE
#define KERNEL_PREFIX "#pragma OPENCL EXTENSION cl_khr_fp64: enable\n#define make_double4(a) (double4)(a)\n\n"
#define KERNEL_OPTIONS "-D dDOUBLE -cl-fast-relaxed-math"
#endif

#elif defined(USE_OPENMP)

#include <vector_types.h>
#include "openmp_common.h"
#define PARALLEL_TYPE OpenMP
#define PARALLEL_ENABLED 1

#else

#define PARALLEL_TYPE None
#define PARALLEL_ENABLED 0

#endif

#if PARALLEL_ENABLED
#include "parallel_defines.h"
#endif

namespace ParallelOptions
{
enum ParallelOption {
  MAXBATCHES = 4,
  MAXBODYREPETITION = 31,
  BSIZE = 64,
  MBSIZE = 96,
  MAXTHREADS = 512,
  SHAREDMEMORYSIZE = 16384,
  MEMALIGN = 128,
  DEFAULTALIGN = 32,
  FLOATALIGN = MEMALIGN / sizeof(float),
  DOUBLEALIGN = MEMALIGN / sizeof(double),
  DEFAULTSCALARALIGN = FLOATALIGN };
}
typedef ParallelOptions::ParallelOption ParallelOption;

typedef size_t ArraySize;
typedef size_t ArrayIndex;

typedef std::vector<int> IntVector;
typedef IntVector::iterator IntVectorIter;
typedef IntVector::const_iterator ConstIntVectorIter;

#define PROJ_NAME "parallel_quickstep"

#endif
