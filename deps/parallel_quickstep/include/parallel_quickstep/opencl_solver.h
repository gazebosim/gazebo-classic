#ifndef OPENCL_SOLVER_H
#define OPENCL_SOLVER_H

#include "parallel_solver.h"

#include <string>

namespace parallel_ode
{

static const int DEFAULT_CL_FLAGS =
    ParallelFlags::PARALLEL_ALIGN
    // | ParallelFlags::PARALLEL_PINNED
    | ParallelFlags::PARALLEL_REDUCE
    | ParallelFlags::PARALLEL_ASYNC
    | ParallelFlags::PARALLEL_RANDOMIZE;

template<typename T>
class OpenCLPGSSolver : public ParallelPGSSolver<T, T, ParallelTypes::OpenCL>
{
public:
  typedef typename vec4<T>::Type Vec4T;
  typedef const T* TPtr;
  typedef T* TMutablePtr;

  OpenCLPGSSolver( int parallelFlags = DEFAULT_CL_FLAGS,
                   BatchType batchType = BatchTypes::DEFAULT_BATCH_TYPE,
                   ReduceType reduceType = ReduceTypes::REDUCE_STRIDED,
                   uint numBatches = ParallelOptions::MAXBATCHES )

      : ParallelPGSSolver<T, T, ParallelTypes::OpenCL>( parallelFlags, batchType, reduceType, numBatches ) {

  }

  virtual ~OpenCLPGSSolver( ) {

  }

  virtual void initialize( );

protected:

  virtual void solveAndReduce( const int offset, const int batchSize );

  virtual void loadConstraints( );

  virtual void loadKernels( );

};

}

#endif
