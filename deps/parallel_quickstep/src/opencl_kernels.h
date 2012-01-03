#ifndef OPENCL_KERNELS_H
#define OPENCL_KERNELS_H

#include <parallel_common.h>

class ReduceStrategy;

namespace parallel_ode
{

extern "C" void oclInitializeKernels(cl_context cxGPUContext, cl_command_queue cqCommandQueue);
extern "C" void oclShutdownKernels(void);

void oclZeroVector( cl_mem buffer, int bufferSize, bool bScalarType = false );

void oclPGSReduce( cl_mem fc0_reduction,
                   cl_mem fc1_reduction,
                   ReduceStrategy* reduceStrategy );

void oclPGSReduce( cl_mem fc0,
                   cl_mem fc1,
                   cl_mem fc0_reduction,
                   cl_mem fc1_reduction,
                   ReduceStrategy* reduceStrategy );

void oclPGSSolve( int offset, int numConstraints, bool bUseAtomics );

void oclPGSSolveInit( cl_mem bodyIDs,
                      cl_mem fIDs,
                      cl_mem j,
                      cl_mem ij,
                      cl_mem fc0,
                      cl_mem fc1,
                      cl_mem fc0_reduction,
                      cl_mem fc1_reduction,
                      cl_mem lambda,
                      cl_mem adcfm,
                      cl_mem rhs,
                      cl_mem hilo,
                      int bStride, int cStride,
                      ReduceStrategy* reduceStrategy );

}

#endif
