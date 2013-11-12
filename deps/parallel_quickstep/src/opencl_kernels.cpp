#include "opencl_kernels.h"

#include <opencl_utils.h>
#include <parallel_common.h>
#include <parallel_reduce.h>
#include <parallel_math.h>

namespace parallel_ode
{

static cl_program cpSolver;             /**< OpenCL solver program */

/**
 * Aligns a to the next largest multiple of b
 *
 * @param a the parameter to align
 * @param b the alignment
 *
 * @return the aligned size
 */
static size_t uSnap(size_t a, size_t b){
  return ((a % b) == 0) ? a : (a - (a % b) + b);
}

static cl_kernel
  ckReduce,                             /**< The reduction kernel */
  ckZero,                               /**< The clearing kernel for scalar values */
  ckZero4,                              /**< The clearing kernel for vec4 values */
  ckSolve;                              /**< The solution kernel */

static cl_command_queue cqDefaultCommandQueue; /**< Local storage of global command queue */

/**
 * Extracts the directory from a given ptah
 *
 * @param path The global path of a file
 *
 * @return A string containing the directory
 */
std::string extractDirectory( const std::string& path )
{
  return path.substr( 0, path.find_last_of( '/' )+1 );
}

static const std::string FILENAME =  __FILE__; /**< This file name, as a global path */
static const std::string SRC_DIR = extractDirectory( FILENAME ); /**< The directory of this file */
static const std::string INC_DIR = extractDirectory( FILENAME ) + "../include/" + PROJ_NAME + "/"; /**< The directory of includes */

extern "C" void oclInitializeKernels(cl_context cxGPUContext, cl_command_queue cqParamCommandQue)
{
  cl_int ciErrNum;
  size_t kernelLength;

  char *cDefines1 = oclLoadProgSource((INC_DIR+"parallel_defines.h").c_str(), KERNEL_PREFIX, &kernelLength);
  oclCheckError(cDefines1 != NULL, true);
  char *cDefines2 = oclLoadProgSource((INC_DIR+"opencl_common.h").c_str(), cDefines1, &kernelLength);
  oclCheckError(cDefines2 != NULL, true);
  char *cKernels = oclLoadProgSource((SRC_DIR+"parallel_kernels_nontemplate.h").c_str(), cDefines2, &kernelLength);
  //char *cKernels = oclLoadProgSource((SRC_DIR+"parallel_kernels.cl").c_str(), cDefines1, &kernelLength);
  oclCheckError(cKernels != NULL, true);

  cpSolver = clCreateProgramWithSource(cxGPUContext, 1, (const char **)&cKernels, &kernelLength, &ciErrNum);
  oclCheckError(ciErrNum, CL_SUCCESS);

  ciErrNum = clBuildProgram(cpSolver, 0, NULL, KERNEL_OPTIONS, NULL, NULL);
  if (ciErrNum != CL_SUCCESS)
  {
    oclLogBuildInfo(cpSolver, oclGetFirstDev(cxGPUContext));
    oclLogPtx(cpSolver, oclGetFirstDev(cxGPUContext), "clSolverError.ptx");
    oclCheckError(ciErrNum, CL_SUCCESS);
  }

  ckReduce = clCreateKernel(cpSolver, "parallelReduce", &ciErrNum);
  oclCheckError(ciErrNum, CL_SUCCESS);
  ckZero = clCreateKernel(cpSolver, "parallelZero", &ciErrNum);
  oclCheckError(ciErrNum, CL_SUCCESS);
  ckZero4 = clCreateKernel(cpSolver, "parallelZero4", &ciErrNum);
  oclCheckError(ciErrNum, CL_SUCCESS);
  ckSolve = clCreateKernel(cpSolver, "parallelSORLCP", &ciErrNum);
  oclCheckError(ciErrNum, CL_SUCCESS);

  //Save default command queue
  cqDefaultCommandQueue = cqParamCommandQue;

  //Discard temp storage
  free(cDefines1);
  free(cDefines2);
  free(cKernels);

  //Save ptx code to separate file
  oclLogPtx(cpSolver, oclGetFirstDev(cxGPUContext), "oclSolver.ptx");
}

extern "C" void oclShutdownKernels(void) {
  cl_int ciErrNum = CL_SUCCESS;
  ciErrNum |= clReleaseKernel(ckReduce);
  ciErrNum |= clReleaseKernel(ckSolve);
  ciErrNum |= clReleaseKernel(ckZero);
  ciErrNum |= clReleaseKernel(ckZero4);
  ciErrNum |= clReleaseProgram(cpSolver);
  oclCheckError(ciErrNum, CL_SUCCESS);
}

void oclPGSReduce( cl_mem fc0_reduction,
                   cl_mem fc1_reduction,
                   ReduceStrategy* reduceStrategy )
{
  cl_int ciErrNum = CL_SUCCESS;

  const int bodyReductionSize = reduceStrategy->getBodySizeWithReduction( );
  const size_t bodySize = reduceStrategy->getBodySize( );
  const size_t localWorkSize = 16;
  const size_t globalWorkSize = uSnap(bodySize, localWorkSize);

  switch( reduceStrategy->getType( ) )
  {
    case ReduceTypes::REDUCE_STRIDED:
      {
        ciErrNum = clEnqueueNDRangeKernel(cqDefaultCommandQueue, ckReduce, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCheckError(ciErrNum, CL_SUCCESS);
      }
      break;
    case ReduceTypes::REDUCE_SEQUENTIAL:
    case ReduceTypes::REDUCE_COMPACT:
    case ReduceTypes::REDUCE_NONE:
    default:
      oclCheckError(0, 1);
      break;

  };

  if( reduceStrategy->clearReduceBuffers( ) ) {
    oclZeroVector(fc0_reduction, bodyReductionSize, false);
    oclZeroVector(fc1_reduction, bodyReductionSize, false);
  }
}

void oclPGSReduce( cl_mem fc0,
                   cl_mem fc1,
                   cl_mem fc0_reduction,
                   cl_mem fc1_reduction,
                   ReduceStrategy* reduceStrategy )
{
  cl_int ciErrNum = CL_SUCCESS;

  const int bodySize = reduceStrategy->getBodySize( );
  const int bodyReductionSize = reduceStrategy->getBodySizeWithReduction( );
  const int bodyOffsetStride = reduceStrategy->getBodyOffsetStride( );

  ciErrNum  = clSetKernelArg(ckReduce, 0, sizeof(cl_mem), (void *)&fc0);
  ciErrNum |= clSetKernelArg(ckReduce, 1, sizeof(cl_mem), (void *)&fc1);
  ciErrNum |= clSetKernelArg(ckReduce, 2, sizeof(cl_mem), (void *)&fc0_reduction);
  ciErrNum |= clSetKernelArg(ckReduce, 3, sizeof(cl_mem), (void *)&fc1_reduction);
  ciErrNum |= clSetKernelArg(ckReduce, 4, sizeof(int), (void *)&bodyOffsetStride);
  ciErrNum |= clSetKernelArg(ckReduce, 5, sizeof(int), (void *)&bodySize);
  ciErrNum |= clSetKernelArg(ckReduce, 6, sizeof(int), (void *)&bodyReductionSize);
  oclCheckError(ciErrNum, CL_SUCCESS);

  oclPGSReduce(fc0_reduction, fc1_reduction, reduceStrategy);
}

void oclZeroVector( cl_mem buffer, int bufferSize, bool bScalarType )
{
  const size_t localWorkSize = ParallelOptions::BSIZE;
  const size_t globalWorkSize = uSnap(bufferSize, localWorkSize);
  cl_int ciErrNum = CL_SUCCESS;

  if( bScalarType ) {
    ciErrNum  = clSetKernelArg(ckZero, 0, sizeof(cl_mem), (void *)&buffer);
    ciErrNum |= clSetKernelArg(ckZero, 1, sizeof(int), (void *)&bufferSize);
    oclCheckError(ciErrNum, CL_SUCCESS);

    ciErrNum = clEnqueueNDRangeKernel(cqDefaultCommandQueue, ckZero, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
    oclCheckError(ciErrNum, CL_SUCCESS);
  } else {
    ciErrNum  = clSetKernelArg(ckZero4, 0, sizeof(cl_mem), (void *)&buffer);
    ciErrNum |= clSetKernelArg(ckZero4, 1, sizeof(int), (void *)&bufferSize);
    oclCheckError(ciErrNum, CL_SUCCESS);

    ciErrNum = clEnqueueNDRangeKernel(cqDefaultCommandQueue, ckZero4, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
    oclCheckError(ciErrNum, CL_SUCCESS);
  }
}

void oclPGSSolve( int offset, int numConstraints, bool bUseAtomics )
{
  cl_int ciErrNum = CL_SUCCESS;
  size_t globalWorkSizeUnaligned = numConstraints;
  const size_t localWorkSize = ParallelOptions::BSIZE;
  const size_t globalWorkSize = uSnap(globalWorkSizeUnaligned, localWorkSize);

  ciErrNum |= clSetKernelArg(ckSolve, 12, sizeof(int), (void *)&offset);
  ciErrNum |= clSetKernelArg(ckSolve, 13, sizeof(int), (void *)&numConstraints);
  oclCheckError(ciErrNum, CL_SUCCESS);

  ciErrNum = clEnqueueNDRangeKernel(cqDefaultCommandQueue, ckSolve, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
  oclCheckError(ciErrNum, CL_SUCCESS);
}

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
                      ReduceStrategy* reduceStrategy )
{
  cl_int ciErrNum = CL_SUCCESS;

  // Initialize ckSolve kernel arguments
  ciErrNum |= clSetKernelArg(ckSolve, 0, sizeof(cl_mem), (void *)&fc0_reduction);
  ciErrNum |= clSetKernelArg(ckSolve, 1, sizeof(cl_mem), (void *)&fc1_reduction);
  ciErrNum |= clSetKernelArg(ckSolve, 2, sizeof(cl_mem), (void *)&lambda);
  ciErrNum |= clSetKernelArg(ckSolve, 3, sizeof(cl_mem), (void *)&bodyIDs);
  ciErrNum |= clSetKernelArg(ckSolve, 4, sizeof(cl_mem), (void *)&fIDs);
  ciErrNum |= clSetKernelArg(ckSolve, 5, sizeof(cl_mem), (void *)&j);
  ciErrNum |= clSetKernelArg(ckSolve, 6, sizeof(cl_mem), (void *)&ij);
  ciErrNum |= clSetKernelArg(ckSolve, 7, sizeof(cl_mem), (void *)&fc0);
  ciErrNum |= clSetKernelArg(ckSolve, 8, sizeof(cl_mem), (void *)&fc1);
  ciErrNum |= clSetKernelArg(ckSolve, 9, sizeof(cl_mem), (void *)&adcfm);
  ciErrNum |= clSetKernelArg(ckSolve, 10, sizeof(cl_mem), (void *)&rhs);
  ciErrNum |= clSetKernelArg(ckSolve, 11, sizeof(cl_mem), (void *)&hilo);
  ciErrNum |= clSetKernelArg(ckSolve, 14, sizeof(int), (void *)&bStride);
  ciErrNum |= clSetKernelArg(ckSolve, 15, sizeof(int), (void *)&cStride);
  oclCheckError(ciErrNum, CL_SUCCESS);

  // Initialize ckReduce kernel arguments
  const int bodySize = reduceStrategy->getBodySize( );
  const int bodyReductionSize = reduceStrategy->getBodySizeWithReduction( );
  const int bodyOffsetStride = reduceStrategy->getBodyOffsetStride( );

  ciErrNum  = clSetKernelArg(ckReduce, 0, sizeof(cl_mem), (void *)&fc0);
  ciErrNum |= clSetKernelArg(ckReduce, 1, sizeof(cl_mem), (void *)&fc1);
  ciErrNum |= clSetKernelArg(ckReduce, 2, sizeof(cl_mem), (void *)&fc0_reduction);
  ciErrNum |= clSetKernelArg(ckReduce, 3, sizeof(cl_mem), (void *)&fc1_reduction);
  ciErrNum |= clSetKernelArg(ckReduce, 4, sizeof(int), (void *)&bodyOffsetStride);
  ciErrNum |= clSetKernelArg(ckReduce, 5, sizeof(int), (void *)&bodySize);
  ciErrNum |= clSetKernelArg(ckReduce, 6, sizeof(int), (void *)&bodyReductionSize);
  oclCheckError(ciErrNum, CL_SUCCESS);
}

}
