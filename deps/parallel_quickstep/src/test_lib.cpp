#include <parallel_array.h>
#include <parallel_common.h>
#include <parallel_math.h>
#include <parallel_reduce.h>

#include <iostream>
#include <stdio.h>

#ifdef USE_CUDA
#include "cuda_kernels.cuh"
#elif USE_OPENCL
#include "opencl_kernels.h"
#endif

using namespace parallel_utils;
using namespace parallel_ode;

template <typename T, ParallelType PType>
int parallelReduceStrategyTest( ReduceType reduceType )
{
  typedef typename vec4<T>::Type Vec4T;
  typedef ParallelHDArray<Vec4T,PType> ParallelHDArray;

  dxInitDevice();

  const ArraySize reduceStride = 32;
  const ArraySize reduceSize = 2;
  const ArraySize reduceBufferSize = reduceSize * reduceStride;

  ParallelHDArray testArrayHD0( reduceBufferSize );
  ParallelHDArray testArrayHD1( reduceBufferSize );

  Vec4T* testArrayHP0 = testArrayHD0.getHostBuffer( );
  Vec4T* testArrayHP1 = testArrayHD1.getHostBuffer( );

  for( size_t i = 0; i < reduceBufferSize; i++) {
    testArrayHP0[ i ] = make_vec4( (T)i );
    testArrayHP1[ i ] = make_vec4( (T)i );
  }

  testArrayHD0.syncToDevice( CopyTypes::COPY_SYNC );
  testArrayHD1.syncToDevice( CopyTypes::COPY_SYNC );

  parallel_ode::ReduceStrategy* reduceStrategy = parallel_ode::ReduceStrategyFactory::create( reduceType, false );
  std::vector<int> reduceRepetitionCount;
  switch( reduceType ) {
    case ReduceTypes::REDUCE_STRIDED:
      reduceStrategy->initialize( reduceStride, reduceSize, reduceRepetitionCount );
      break;
    case ReduceTypes::REDUCE_SEQUENTIAL:
      reduceStrategy->initialize( reduceSize, reduceStride, reduceRepetitionCount );
      break;
    default:
      reduceStrategy->initialize( reduceStride, reduceSize, reduceRepetitionCount );
      break;
  }

#ifdef USE_CUDA
  cudaPGSReduce<T>( testArrayHD0.getDeviceBuffer( ), testArrayHD1.getDeviceBuffer( ),
                    testArrayHD0.getDeviceBuffer( ), testArrayHD1.getDeviceBuffer( ), reduceStrategy );
#elif USE_OPENCL
  oclPGSReduce( testArrayHD0.getDeviceBuffer( ), testArrayHD1.getDeviceBuffer( ),
                testArrayHD0.getDeviceBuffer( ), testArrayHD1.getDeviceBuffer( ), reduceStrategy );
#endif
  testArrayHD0.syncToHost( CopyTypes::COPY_SYNC );
  testArrayHD1.syncToHost( CopyTypes::COPY_SYNC );

  testArrayHD0.print( "TestReduceArray" );
  testArrayHD1.print( "TestReduceArray2");

  for( size_t i = 0; i < reduceBufferSize; i++) {
    testArrayHP1[ i ] = make_vec4( (T)0.0 );
  }

#ifdef USE_CUDA
  cudaZeroVector<Vec4T>( testArrayHD0.getDeviceBuffer( ), reduceBufferSize );
#elif USE_OPENCL
  oclZeroVector( testArrayHD0.getDeviceBuffer( ), reduceBufferSize );
#endif

  testArrayHD1.syncToDevice( CopyTypes::COPY_SYNC );
  testArrayHD1.syncToHost( CopyTypes::COPY_SYNC );
  testArrayHD0.syncToHost( CopyTypes::COPY_SYNC );

  testArrayHD0.print( "TestZeroArray" );
  testArrayHD1.print( "TestZeroArray2" );

  delete reduceStrategy;
  dxShutdownDevice();
  return 0;
}

template <typename T, ParallelType PType>
int parallelReduceTest()
{
  parallelReduceStrategyTest<T,PType>( ReduceTypes::REDUCE_STRIDED );
  return 0;
}

#ifdef CUDA_DOUBLESUPPORT
template int parallelReduceTest<dReal,ParallelTypes::PARALLEL_TYPE>();
#else
template int parallelReduceTest<float,ParallelTypes::PARALLEL_TYPE>();
#endif

