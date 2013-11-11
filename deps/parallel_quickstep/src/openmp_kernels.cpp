#include <parallel_common.h>
#include <parallel_reduce.h>
#include <parallel_math.h>

#include "openmp_kernels.h"
#include "openmp_utils.h"

#include <omp.h>
#include <limits>

#include "parallel_kernels.h"

namespace parallel_ode
{

template <typename T>
void ompZeroVector( T *buffer, int bufferSize )
{
  //#pragma omp parallel for
  for(int index = 0; index < bufferSize; ++index) {
    buffer[ index ] = parallel_zero<T>();
  }

}

template <typename T>
void ompPGSReduce( typename vec4<T>::Type *fc0,
                   typename vec4<T>::Type *fc1,
                   typename vec4<T>::Type *fc0_reduction,
                   typename vec4<T>::Type *fc1_reduction,
                   ReduceStrategy* reduceStrategy )
{
  typedef typename vec4<T>::Type Vec4T;

  const int bodySize = reduceStrategy->getBodySize( );
  const int reductionSize = reduceStrategy->getBodySizeWithReduction( );
  const int reductionStride = reduceStrategy->getBodyOffsetStride( );

  const Vec4T zeroVec = make_vec4( (T)0.0 );

  //#pragma omp parallel for
  for(int index = 0; index < bodySize; ++index) {
    int nextIndex = index + reductionStride;

    Vec4T sum0 = fc0_reduction[ index ];
    Vec4T sum1 = fc1_reduction[ index ];

    while(nextIndex < reductionSize) {
      sum0 += fc0_reduction[ nextIndex ];
      fc0_reduction[ nextIndex ] = zeroVec;
      sum1 += fc1_reduction[ nextIndex ];
      fc1_reduction[ nextIndex ] = zeroVec;
      nextIndex += reductionStride;
    }

    fc0[ index ] += sum0;
    fc1[ index ] += sum1;
  }

}

template <typename T>
void ompPGSSolve( int4 *bodyIDs,
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
  typedef typename vec4<T>::Type Vec4T;

  //#pragma omp parallel for
  for(int localIndex = 0; localIndex < numConstraints; ++localIndex) {

    const int index = localIndex + offset;

    T old_lambda = lambda[ index ];

    int4 bodyID = bodyIDs[ index ];

    Vec4T fc00 = fc0[ bodyID.x ];
    Vec4T fc01 = fc1[ bodyID.x ];
    Vec4T fc10 = make_vec4( (T)0.0 );
    Vec4T fc11 = make_vec4( (T)0.0 );

    Vec4T j0_temp = j[      index    ];
    Vec4T j1_temp = j[ C_ID(index,1) ];
    Vec4T j2_temp = j[ C_ID(index,2) ];
    Vec4T j3_temp = j[ C_ID(index,3) ];

    T  delta = rhs[ index ] - old_lambda * adcfm[ index ];

    if( bodyID.y >= 0 )  {
      fc10 = fc0[ bodyID.y ];
      fc11 = fc1[ bodyID.y ];
    }

    {
      delta -= dot( fc00, j0_temp );
      delta -= dot( fc01, j1_temp );
      if (bodyID.y >= 0) {
        delta -= dot( fc10, j2_temp );
        delta -= dot( fc11, j3_temp );
      }
    }

    {
      T lo_act = hilo[ index ];
      T hi_act = hilo[ C_ID(index,1) ];
      int fID = fIDs[ index ];

      if (fID >= 0) {
        hi_act = abs( hi_act * lambda[ fID ]);
        lo_act = -hi_act;
      }

      T new_lambda = old_lambda + delta;
      T final_lambda = new_lambda;

      if (new_lambda < lo_act) {
        delta = lo_act-old_lambda;
        final_lambda = lo_act;
      }
      else if (new_lambda > hi_act) {
        delta = hi_act-old_lambda;
        final_lambda = hi_act;
      }
      lambda[ index ] = final_lambda;
    }

    j0_temp = ij[      index    ];
    j1_temp = ij[ C_ID(index,1) ];
    j2_temp = ij[ C_ID(index,2) ];
    j3_temp = ij[ C_ID(index,3) ];

    {
      j0_temp *= delta;
      j1_temp *= delta;

      // STORE
      if( bUseAtomics ) {
        myAtomicVecAdd<T>(fc0[ bodyID.x ], j0_temp);
        myAtomicVecAdd<T>(fc1[ bodyID.x ], j1_temp);
      } else {
        fc0_reduction[ bodyID.z ] = j0_temp;
        fc1_reduction[ bodyID.z ] = j1_temp;
      }

      if( bodyID.y >= 0 ) {
        j2_temp *= delta;
        j3_temp *= delta;

        // STORE
        if( bUseAtomics ) {
          myAtomicVecAdd<T>(fc0[ bodyID.y ], j2_temp);
          myAtomicVecAdd<T>(fc1[ bodyID.y ], j3_temp);
        } else {
          fc0_reduction[ bodyID.w ] = j2_temp;
          fc1_reduction[ bodyID.w ] = j3_temp;
        }
      }
    }
  }
}


// Explicit specializations needed to generate code
template void ompZeroVector<dReal4>( dReal4  *buffer,
                                     int bufferSize );

template void ompPGSReduce<dReal>( dReal4 *fc0,
                                   dReal4 *fc1,
                                   dReal4 *fc0_reduction,
                                   dReal4 *fc1_reduction,
                                   ReduceStrategy* reduceStrategy );

template void ompPGSSolve<dReal>( int4 *bodyIDs,
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
                                  int batch, int numConstraints, bool bUseAtomics, int bStride, int cStride );

}
