#ifndef OPENMP_SOLVER_H
#define OPENMP_SOLVER_H

#include "parallel_solver.h"

namespace parallel_ode
{

const int DEFAULT_OPENMP_FLAGS =
    ParallelFlags::PARALLEL_ALIGN
    | ParallelFlags::PARALLEL_RANDOMIZE
    | ParallelFlags::PARALLEL_ATOMICS;

template<typename T>
class OpenMPPGSSolver : public ParallelPGSSolver<T, T, ParallelType::OpenMP>
{
public:
  typedef typename vec4<T>::Type Vec4T;

  /**
   * @brief Constructs an instance of an OpenMP-based quickstep solver
   *
   * @param parallelFlags Flags controller various parallel functionality
   * @param batchType The type of batch strategy to be used
   * @param reduceType The Type of reduction to be used
   * @param numBatches The maximum number of batches to be used, if any
   */
  OpenMPPGSSolver( int parallelFlags = DEFAULT_OPENMP_FLAGS,
                BatchType batchType = BatchTypes::DEFAULT_BATCH_TYPE,
                ReduceType reduceType = ReduceTypes::DEFAULT_REDUCE_TYPE,
                uint numBatches = ParallelOptions::MAXBATCHES )

      : ParallelPGSSolver<T, T, ParallelTypes::OpenMP>( parallelFlags, batchType, reduceType, numBatches ) {

  }

  virtual ~OpenMPPGSSolver( ) {

  }

protected:

  virtual void solveAndReduce( const int offset, const int batchSize );

  virtual void loadConstraints( );
};

}

#endif
