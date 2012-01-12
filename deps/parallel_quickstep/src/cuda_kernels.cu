#include <cuda_utils.h>
#include <parallel_common.h>
#include <parallel_utils.h>
#include <parallel_reduce.h>

#include "cuda_kernels.cuh"
#include "cuda_utils.cuh"
#include <math.h>

#include "parallel_kernels.h"
#include "parallel_kernels_nontemplate.h"

namespace parallel_ode
{

using ::parallel_utils::computeGridSize;

__device__ __constant__ int4 B_C_BSTRIDE_CSTRIDE;

void cudaPGSLoadConstants( int numConstraints,
                           int constraintStride,
                           int constraintVecStride,
                           int numBodies,
                           int bodyStride,
                           int bodyVecStride )
{
  int4 b_c_bstride_cstride = make_int4( numBodies, numConstraints, bodyStride, constraintStride );
  CUDA_SAFE_CALL( cudaMemcpyToSymbol(B_C_BSTRIDE_CSTRIDE, &b_c_bstride_cstride, sizeof(int4)) );
}

template <typename T>
void cudaPGSReduce( typename vec4<T>::Type *fc0,
                    typename vec4<T>::Type *fc1,
                    typename vec4<T>::Type *fc0_reduction,
                    typename vec4<T>::Type *fc1_reduction,
                    ReduceStrategy* reduceStrategy )
{
  typedef typename vec4<T>::Type Vec4T;

  int numBlocks, numThreads;

  const int bodySize = reduceStrategy->getBodySize( );
  const int bodyReductionSize = reduceStrategy->getBodySizeWithReduction( );
  const int bodyStride = reduceStrategy->getBodyStride( );
  const int bodyOffsetStride = reduceStrategy->getBodyOffsetStride( );

  int smemSize = 0;

  switch( reduceStrategy->getType( ) )
  {
    case ReduceTypes::REDUCE_SEQUENTIAL:
    {
      smemSize = (bodyStride<=32)?4*bodyStride*sizeof(Vec4T):2*bodyStride*sizeof(Vec4T);

      switch( bodyStride )
      {
        case 128:
          cudaReduceSequentialT<T, 128><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case 64:
          cudaReduceSequentialT<T, 64><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case 32:
          cudaReduceSequentialT<T, 32><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case 16:
          cudaReduceSequentialT<T, 16><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case  8:
          cudaReduceSequentialT<T, 8><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case  4:
          cudaReduceSequentialT<T, 4><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case  2:
          cudaReduceSequentialT<T, 2><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
        case  1:
          cudaReduceSequentialT<T, 1><<< bodySize, bodyStride, smemSize >>>( fc0, fc1, fc0_reduction, fc1_reduction, bodyReductionSize ); break;
      }
    } break;
    case ReduceTypes::REDUCE_STRIDED:
      computeGridSize( bodySize, 16, numBlocks, numThreads);
      dxExecKernel(numBlocks,
                   numThreads,
                   (cudaReduceStridedT<T>),
                   ( fc0, fc1, fc0_reduction, fc1_reduction, bodyOffsetStride, bodySize, bodyReductionSize ) );
      break;
    case ReduceTypes::REDUCE_COMPACT:
      computeGridSize(bodyReductionSize, ParallelOptions::BSIZE, numBlocks, numThreads);
      for( int treePower = (int)log2( (float)bodyOffsetStride ); treePower > 0; treePower /= 2) {
          dxExecKernel(numBlocks,
                       numThreads,
                       (cudaReduceIterativeCompactT<T>),
                       ( fc0_reduction, fc1_reduction, treePower ) );
      }
      break;
    case ReduceTypes::REDUCE_NONE:    break;

  };

  if( reduceStrategy->clearReduceBuffers( ) ) {
    cudaZeroVector<Vec4T>(fc0_reduction, bodyReductionSize);
    cudaZeroVector<Vec4T>(fc1_reduction, bodyReductionSize);
  }

  CUDA_CHECK_ERROR_BASE();
}

template <typename T>
void cudaZeroVector( T *buffer, int bufferSize )
{
  int numThreads, numBlocks;
  computeGridSize(bufferSize, ParallelOptions::BSIZE, numBlocks, numThreads);

  dxExecKernel( numBlocks,
                numThreads,
                (cudaZeroT<T>),
                ( buffer, bufferSize ) );

  CUDA_CHECK_ERROR_BASE();
}

template <typename T>
void cudaPGSSolve( int4 *bodyIDs,
                   int  *fIDs,
                   typename vec4<T>::Type *j,
                   typename vec4<T>::Type *ij,
                   typename vec4<T>::Type *fc0,
                   typename vec4<T>::Type *fc1,
                   typename vec4<T>::Type *fc0_reduction,
                   typename vec4<T>::Type *fc1_reduction,
                   T* lambda,
                   T* adcfm,
                   T* rhs,
                   T* hilo,
                   int offset, int numConstraints, bool bUseAtomics,
                   int bStride, int cStride)
{
  int numThreads, numBlocks;
  computeGridSize(numConstraints, ParallelOptions::BSIZE, numBlocks, numThreads);

  dxExecKernel(numBlocks,
               numThreads,
               (cudaSORLCPT<T>),
               ( fc0_reduction,
                 fc1_reduction,
                 lambda,
                 bodyIDs,
                 fIDs,
                 j,
                 ij,
                 fc0,
                 fc1,
                 adcfm,
                 rhs,
                 hilo,
                 offset,
                 numConstraints,
                 bStride,
                 cStride
                 ) );

  CUDA_CHECK_ERROR_BASE();
}

template <typename T>
void cudaPGSPreprocess( int4 *bodyIDs,
                        typename vec4<T>::Type *j0,
                        typename vec4<T>::Type *j1,
                        typename vec4<T>::Type *j2,
                        typename vec4<T>::Type *j3,
                        typename vec4<T>::Type *ij0,
                        typename vec4<T>::Type *ij1,
                        typename vec4<T>::Type *ij2,
                        typename vec4<T>::Type *ij3,
                        typename vec4<T>::Type *i0,
                        typename vec4<T>::Type *i1,
                        typename vec4<T>::Type *i2,
                        T* iMass,
                        T* adcfm,
                        T* rhs,
                        T sorParam, T deltaTime, int numConstraints )
{
  int numThreads, numBlocks;
  computeGridSize(numConstraints, ParallelOptions::BSIZE, numBlocks, numThreads);

  dxExecKernel(numBlocks,
               numThreads,
               (cudaComputeInvMJTT<T>),
               (bodyIDs, j0, j1, j2, j3, ij0, i0, i1, i2, iMass, numConstraints, ij1, ij2, ij3 ));

  dxExecKernel(numBlocks,
               numThreads,
               (cudaComputeAdcfmBT<T>),
               ( bodyIDs, j0, j1, j2, j3, ij0, ij1, ij2, ij3, adcfm, rhs, sorParam, numConstraints ));


  CUDA_CHECK_ERROR_BASE();

}

#ifdef CUDA_DOUBLESUPPORT
template void cudaPGSReduce<dReal>( dReal4 *fc0,
                                    dReal4 *fc1,
                                    dReal4 *fc0_reduction,
                                    dReal4 *fc1_reduction,
                                    ReduceStrategy* reduceStrategy );

template void cudaPGSSolve<dReal>( int4 *bodyIDs,
                                   int *fIDs,
                                   dReal4 *j,
                                   dReal4 *ij,
                                   dReal4 *fc0,
                                   dReal4 *fc1,
                                   dReal4 *fc0_reduction,
                                   dReal4 *fc1_reduction,
                                   dReal *lambda,
                                   dReal *adcfm,
                                   dReal *rhs,
                                   dReal *hilo,
                                   int offset, int numConstraints, bool bUseAtomics, int bStride, int cStride );

template void cudaPGSPreprocess<dReal>( int4 *bodyIDs,
                                        dReal4 *j0,
                                        dReal4 *j1,
                                        dReal4 *j2,
                                        dReal4 *j3,
                                        dReal4 *ij0,
                                        dReal4 *ij1,
                                        dReal4 *ij2,
                                        dReal4 *ij3,
                                        dReal4 *ii0,
                                        dReal4 *ii1,
                                        dReal4 *ii2,
                                        dReal *adcfm,
                                        dReal *ad,
                                        dReal *rhs,
                                        dReal sorParam, dReal deltaTime, int numConstraints );

template void cudaZeroVector<dReal4>( dReal4 *buffer, int bufferSize );
#else
template void cudaPGSReduce<float>( float4 *fc0,
                                    float4 *fc1,
                                    float4 *fc0_reduction,
                                    float4 *fc1_reduction,
                                    ReduceStrategy* reduceStrategy );

template void cudaPGSSolve<float>( int4 *bodyIDs,
                                   int *fIDs,
                                   float4 *j,
                                   float4 *ij,
                                   float4 *fc0,
                                   float4 *fc1,
                                   float4 *fc0_reduction,
                                   float4 *fc1_reduction,
                                   float *lambda,
                                   float *adcfm,
                                   float *rhs,
                                   float *hilo,
                                   int offset, int numConstraints, bool bUseAtomics, int bStride, int cStride );

template void cudaPGSPreprocess<float>( int4 *bodyIDs,
                                        float4 *j0,
                                        float4 *j1,
                                        float4 *j2,
                                        float4 *j3,
                                        float4 *ij0,
                                        float4 *ij1,
                                        float4 *ij2,
                                        float4 *ij3,
                                        float4 *ii0,
                                        float4 *ii1,
                                        float4 *ii2,
                                        float *adcfm,
                                        float *ad,
                                        float *rhs,
                                        float sorParam, float deltaTime, int numConstraints );

template void cudaZeroVector<float4>( float4 *buffer, int bufferSize );
#endif

}
