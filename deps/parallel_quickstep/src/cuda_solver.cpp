#include <cuda_solver.h>
#include <parallel_utils.h>
#include <cuda_utils.h>

#include "cuda_kernels.cuh"

namespace parallel_ode
{

#ifdef CUDA_DOUBLESUPPORT
template class CudaPGSSolver<dReal,dReal>;
#else
template class CudaPGSSolver<float,dReal>;
#endif

template<typename CudaT,typename ParamsT>
void CudaPGSSolver<CudaT,ParamsT>::initialize( )
{
  ParallelPGSSolver<CudaT,ParamsT,ParallelTypes::CUDA>::initialize( );

  MemFlags flags = cudaHostAllocDefault;
  flags |= this->wcMemEnabled() ? cudaHostAllocWriteCombined : 0;
  flags |= this->pinnedMemEnabled() ? cudaHostAllocPortable : 0;
  this->setMemFlags( flags );
}

template<typename CudaT,typename ParamsT>
void CudaPGSSolver<CudaT,ParamsT>::preProcessDevice( const CudaT sorParam, const CudaT stepSize )
{
  /** @todo Finish preprocessing implementation */
}

template<typename CudaT,typename ParamsT>
void CudaPGSSolver<CudaT,ParamsT>::solveAndReduce( const int offset, const int batchSize )
{
  // cuda stuff here should change it's double/single-ness  based on GPU support for doubles
  cudaPGSSolve<CudaT>( this->bodyIDs.getDeviceBuffer( ),
                   this->fIDs.getDeviceBuffer( ),
                   this->j0.getDeviceBuffer( ),
                   this->ij0.getDeviceBuffer( ),
                   this->bodyFAcc.getDeviceBuffer( ),
                   this->bodyTAcc.getDeviceBuffer( ),
                   this->bodyFAccReduction.getDeviceBuffer( ),
                   this->bodyTAccReduction.getDeviceBuffer( ),
                   this->lambda0.getDeviceBuffer( ),
                   this->adcfm.getDeviceBuffer( ),
                   this->rhs.getDeviceBuffer( ),
                   this->lohiD.getDeviceBuffer( ),
                   offset,
                   batchSize,
                   this->atomicsEnabled( ),
                   this->getBodyStride( ),
                   this->getConstraintStride( ) );

  if( this->reduceEnabled( ) ) {
    cudaPGSReduce<CudaT>( this->bodyFAcc.getDeviceBuffer( ),
                      this->bodyTAcc.getDeviceBuffer( ),
                      this->bodyFAccReduction.getDeviceBuffer( ),
                      this->bodyTAccReduction.getDeviceBuffer( ),
                      this->reduceStrategy_ );
  }
}

template<typename CudaT,typename ParamsT>
void CudaPGSSolver<CudaT,ParamsT>::loadConstraints( )
{
  ParallelPGSSolver<CudaT,ParamsT, ParallelTypes::CUDA>::loadConstraints( );

  // Zero out the force accumulation vector
  if( this->reduceEnabled( ) ) {
    cudaZeroVector<Vec4T>(this->bodyFAccReduction.getDeviceBuffer( ), this->bodyFAccReduction.getSize( ));
    cudaZeroVector<Vec4T>(this->bodyTAccReduction.getDeviceBuffer( ), this->bodyTAccReduction.getSize( ));
  }
}

}
