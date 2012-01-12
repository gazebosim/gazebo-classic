#ifndef PARALLEL_DEFINES_H
#define PARALLEL_DEFINES_H

#ifdef dSINGLE
typedef float dReal;
typedef float4 dReal4;
#define make_real3 make_float3
#define make_real4 make_float4
#define dParallelZero 0.0f
#define dParallelInf dxParallelInfF
#elif dDOUBLE
typedef double dReal;
typedef double4 dReal4;
#define make_real3 make_double3
#define make_real4 make_double4
#define dParallelZero 0.0
#define dParallelInf dxParallelInfD
#endif

#endif
