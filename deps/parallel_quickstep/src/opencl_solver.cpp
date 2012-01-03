#include <opencl_solver.h>
#include <parallel_utils.h>

#include "opencl_kernels.h"

namespace parallel_ode
{

template class OpenCLPGSSolver<dReal>;

template<typename T>
void OpenCLPGSSolver<T>::initialize( )
{
  ParallelPGSSolver<T,T,ParallelTypes::OpenCL>::initialize( );
}

template<typename T>
void OpenCLPGSSolver<T>::solveAndReduce( const int offset, const int batchSize )
{
    oclPGSSolve( offset,
                 batchSize,
                 this->atomicsEnabled( ) );

  if( this->reduceEnabled( ) ) {
    oclPGSReduce( this->bodyFAccReduction.getDeviceBuffer( ),
                  this->bodyTAccReduction.getDeviceBuffer( ),
                  this->reduceStrategy_ );
   }
}

template<typename T>
void OpenCLPGSSolver<T>::loadConstraints( )
{
  ParallelPGSSolver<T,T,ParallelTypes::OpenCL>::loadConstraints( );

  // Zero out the force accumulation vector
  if( this->reduceEnabled( ) ) {
    oclZeroVector(this->bodyFAccReduction.getDeviceBuffer( ), this->bodyFAccReduction.getSize( ), false);
    oclZeroVector(this->bodyTAccReduction.getDeviceBuffer( ), this->bodyTAccReduction.getSize( ), false);
  }
}

template<typename T>
void OpenCLPGSSolver<T>::loadKernels( )
{
  oclPGSSolveInit( this->bodyIDs.getDeviceBuffer( ),
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
                   this->getBodyStride( ),
                   this->getConstraintStride( ),
                   this->reduceStrategy_ );

}

}
