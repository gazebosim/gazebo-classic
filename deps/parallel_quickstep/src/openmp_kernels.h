#ifndef OPENMP_KERNELS_H
#define OPENMP_KERNELS_H

class ReduceStrategy;

#include <parallel_common.h>
#include <parallel_math.h>

namespace parallel_ode
{

template <typename T>
void ompZeroVector( T *buffer, int bufferSize );

template <typename T>
void ompPGSReduce( typename vec4<T>::Type *fc0,
                   typename vec4<T>::Type *fc1,
                   typename vec4<T>::Type *fc0_reduction,
                   typename vec4<T>::Type *fc1_reduction,
                   ReduceStrategy* reduceStrategy );

template <typename T>
void ompPGSSolve( int4 *bodyIDs,
                  int *fIDs,
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
                  int offset, int numConstraints, bool bUseAtomics, int bStride, int cStride );

}
#endif
