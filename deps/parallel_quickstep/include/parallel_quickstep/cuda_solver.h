#ifndef CUDA_SOLVER_H
#define CUDA_SOLVER_H

#include "parallel_solver.h"

namespace parallel_ode
{

template<typename CudaT,typename ParamsT>
class CudaPGSSolver : public ParallelPGSSolver<CudaT,ParamsT,ParallelTypes::CUDA>
{
public:
  typedef typename vec4<CudaT>::Type Vec4T;
  typedef const CudaT* CudaTPtr;
  typedef CudaT* CudaTMutablePtr;
  typedef MemManager<CudaT,ParallelTypes::CUDA> PMemManager;
  typedef typename PMemManager::mem_flags MemFlags;

  /**
   * @brief Constructs an instance of an CUDA-based quickstep solver
   *
   * @param parallelFlags Flags controller various parallel functionality
   * @param batchType The type of batch strategy to be used
   * @param reduceType The Type of reduction to be used
   * @param numBatches The maximum number of batches to be used, if any
   */
  CudaPGSSolver( int parallelFlags = DEFAULT_FLAGS,
                 BatchType batchType = BatchTypes::DEFAULT_BATCH_TYPE,
                 ReduceType reduceType = ReduceTypes::DEFAULT_REDUCE_TYPE,
                 uint numBatches = ParallelOptions::MAXBATCHES )

      : ParallelPGSSolver<CudaT,ParamsT,ParallelTypes::CUDA>( parallelFlags, batchType, reduceType, numBatches ) {

  }

  virtual ~CudaPGSSolver( ) {
  }

  /**
   * @brief Overloaded initialization of resources specific to CUDA
   */
  virtual void initialize( );

protected:

  virtual void preProcessDevice( const CudaT sorParam, const CudaT stepsize );

  virtual void solveAndReduce( const int offset, const int batchSize );

  virtual void loadConstraints( );
};

}
#endif
