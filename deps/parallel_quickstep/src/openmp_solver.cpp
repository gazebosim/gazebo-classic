#include <openmp_solver.h>
#include <parallel_utils.h>

#include "openmp_kernels.h"

namespace parallel_ode
{

template class OpenMPPGSSolver<dReal>;

template<typename T>
void OpenMPPGSSolver<T>::solveAndReduce( const int offset, const int batchSize )
{
  ompPGSSolve<T>( this->bodyIDs.getHostBuffer( ),
                  this->fIDs.getHostBuffer( ),
                  this->j0.getHostBuffer( ),
                  this->ij0.getHostBuffer( ),
                  this->bodyFAcc.getHostBuffer( ),
                  this->bodyTAcc.getHostBuffer( ),
                  this->bodyFAccReduction.getHostBuffer( ),
                  this->bodyTAccReduction.getHostBuffer( ),
                  this->lambda0.getHostBuffer( ),
                  this->adcfm.getHostBuffer( ),
                  this->rhs.getHostBuffer( ),
                  this->lohiD.getHostBuffer( ),
                  offset,
                  batchSize,
                  this->atomicsEnabled( ),
                  this->getBodyStride( ),
                  this->getConstraintStride( ) );

  if( this->reduceEnabled( ) ) {
    ompPGSReduce<T>( this->bodyFAcc.getHostBuffer( ),
                     this->bodyTAcc.getHostBuffer( ),
                     this->bodyFAccReduction.getHostBuffer( ),
                     this->bodyTAccReduction.getHostBuffer( ),
                     this->reduceStrategy_ );
  }
}

template<typename T>
void OpenMPPGSSolver<T>::loadConstraints( )
{
  ParallelPGSSolver<T,T,ParallelTypes::OpenMP>::loadConstraints( );

  // Zero out the force accumulation vector
  if( this->reduceEnabled( ) ) {
    ompZeroVector<Vec4T>(this->bodyFAccReduction.getHostBuffer( ), this->bodyFAccReduction.getSize( ));
    ompZeroVector<Vec4T>(this->bodyTAccReduction.getHostBuffer( ), this->bodyTAccReduction.getSize( ));
  }
}

}
