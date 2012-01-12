#ifndef OPENMP_UTILS_H
#define OPENMP_UTILS_H

#include <math.h>

template <typename T>
static inline void myAtomicVecAdd(typename vec4<T>::Type& a, typename vec4<T>::Type& b)
{
  //#pragma omp critical
  a += b;
  /*
#pragma omp atomic
  a.x += b.x;

#pragma omp atomic
  a.y += b.y;

#pragma omp atomic
  a.z += b.z;
  */
}

#endif
