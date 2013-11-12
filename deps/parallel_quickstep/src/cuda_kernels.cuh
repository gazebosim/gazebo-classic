#ifndef CUDA_KERNELS_CUH
#define CUDA_KERNELS_CUH

#include <cuda_runtime.h>
#include <parallel_math.h>

class ReduceStrategy;

namespace parallel_ode
{

void cudaPGSLoadConstants( int numConstraints,
                           int constraintStride,
                           int constraintVecStride,
                           int numBodies,
                           int bodyStride,
                           int bodyVecStride );

template <typename T>
void cudaZeroVector( T *buffer, int bufferSize );

template <typename T>
void cudaPGSReduce( typename vec4<T>::Type *fc0,
                    typename vec4<T>::Type *fc1,
                    typename vec4<T>::Type *fc0_reduction,
                    typename vec4<T>::Type *fc1_reduction,
                    ReduceStrategy* reduceStrategy );

template <typename T>
void cudaPGSSolve( int4 *bodyIDs,
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
                        typename vec4<T>::Type *ii0,
                        typename vec4<T>::Type *ii1,
                        typename vec4<T>::Type *ii2,
                        T* imass,
                        T* adcfm,
                        T* rhs,
                        T sorParam, T deltaTime, int numConstraints );

}

#endif
