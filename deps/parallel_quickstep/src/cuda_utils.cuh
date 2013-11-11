#ifndef CUDA_UTILS_CUH
#define CUDA_UTILS_CUH

#include <cuda_runtime.h>
#include <parallel_math.h>

template <typename T>        __inline__ __device__ void    myAtomicAdd(T *addr, T val) { }

template<typename T> static  __inline__ __device__ T       rsqrt_T(T x) { return rsqrt(x); }
template<>                   __inline__ __device__ float   rsqrt_T<float>(float x) { return rsqrtf(x); }

#ifdef CUDA_ATOMICSUPPORT
#ifdef CUDA_DOUBLESUPPORT

template<> void myAtomicAdd<double>(double *address, double value)
{
  double old=*address, assumed;
  do {
    assumed = old;
    old = __longlong_as_double( atomicCAS((unsigned long long int*)address,
                                          __double_as_longlong(assumed),
                                          __double_as_longlong(value+assumed)));
  } while( __double_as_longlong(assumed)!=__double_as_longlong(old) );
}

__inline__ __device__ void atomicAdd(double *address, double value) {
  myAtomicAdd<double>(address, value);
}


#endif //CUDA_DOUBLESUPPORT

template<> void myAtomicAdd<float>(float* address, float value)
{
  float old = value;
  while ((old = atomicExch(address, atomicExch(address, 0.0f)+old))!=0.0f);
}
#endif // CUDA_ATOMICSUPPORT

template <typename T>
__device__ __inline__ void myAtomicVecAdd(typename vec4<T>::Type& addr, typename vec3<T>::Type& val)
{
//#ifdef CUDA_SM20
#ifdef CUDA_ATOMICSUPPORT
  atomicAdd((T*)&addr, val.x);
  atomicAdd((T*)&addr+1, val.y);
  atomicAdd((T*)&addr+2, val.z);
#else
  addr += make_vec4( val );
  //myAtomicAdd<T>((T*)&addr, val.x);
  //myAtomicAdd<T>((T*)&addr+1, val.y);
  //myAtomicAdd<T>((T*)&addr+2, val.z);
#endif
}

__host__ __device__ bool AlmostEqual2sComplement(float A, float B, int maxUlps)
{
  // Make sure maxUlps is non-negative and small enough that the
  // default NAN won't compare as equal to anything.
  // assert(maxUlps > 0 && maxUlps < 4 * 1024 * 1024);
  int aInt = *(int*)&A;
  // Make aInt lexicographically ordered as a twos-complement int
  if (aInt < 0)
    aInt = 0x80000000 - aInt;
  // Make bInt lexicographically ordered as a twos-complement int
  int bInt = *(int*)&B;
  if (bInt < 0)
    bInt = 0x80000000 - bInt;
  int intDiff = abs(aInt - bInt);
  if (intDiff <= maxUlps)
    return true;
  return false;

}

#endif
