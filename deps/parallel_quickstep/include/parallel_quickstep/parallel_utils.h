#ifndef PARALLEL_UTILS_H
#define PARALLEL_UTILS_H

#include "parallel_common.h"

#include <cstdlib>
#include <cstdio>
#include <string.h>
#include <vector>
#include <limits.h>

namespace parallel_utils
{

#define VectorType ::std::vector

#ifdef VERBOSE
#define IFVERBOSE(x) x
#else
#define IFVERBOSE(x) ((void)0)
#endif

#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif

#ifdef BENCHMARKING
#define IFBENCHMARKING(x) x
#else
#define IFBENCHMARKING(x)
#endif

#ifdef ERROR
#define COMPUTE_ERROR 1
#else
#define COMPUTE_ERROR 0
#endif

/////////////////////////////////////////////////////////////////////////

#define alignedSize(a)                  __alignedSize(a)
#define alignSize(offset,alignment)     (((offset) + (alignment) - 1) & ~ ((alignment) - 1))
#define alignDefaultSize(offset)        alignSize(offset,ParallelOptions::DEFAULTALIGN)
#define alignOffset(offset,alignment)   (offset) = alignSize(offset,alignment)
#define align(offset)                   alignOffset(offset,ParallelOptions::DEFAULTALIGN)

/////////////////////////////////////////////////////////////////////////

inline int __alignedSize( VectorType<int>& vectorToAlign )
{
  int totalSize = 0;
  for( size_t i = 0; i < vectorToAlign.size(); i++ )
  {
    totalSize += vectorToAlign[i];
    align(totalSize);
  }
  return totalSize;
}

inline int iDivUp(int a, int b)
{
  return (a % b != 0) ? (a / b + 1) : (a / b);
}

inline void computeGridSize(int n, int blockSize, int &numBlocks, int &numThreads)
{
  numThreads = std::min(blockSize, n);
  numBlocks = iDivUp(n, numThreads);
}

inline int computeStride(int n, int blockSize)
{
  return iDivUp(n, blockSize) * blockSize;
}

template <typename T>
inline int computeElementsPerAlign( int align )
{
  return align / sizeof( T );
}

template <typename T>
inline T iPower2Up(T k)
{
  --k;
  for (unsigned int i=1; i<sizeof(T)*CHAR_BIT; i<<=1)
    k = k | k >> i;
  return k+1;
}

template <typename T>
inline T iPower2UpUnsigned(T k) {
  if (k == 0)
    return 1;
  --k;
  for (int i=1; i<sizeof(T)*CHAR_BIT; i<<=1)
    k = k | k >> i;
  return k+1;
}

template <typename T>
inline void fillSequentialVector( VectorType<T>& vectorToFill )
{
  for( size_t i = 0; i < vectorToFill.size(); ++i )
    vectorToFill[ i ] = (T)i;
}

template <typename T>
inline void fillStridedVector( VectorType<T>& vectorToFill, T stride )
{
  for( size_t i = 0; i < vectorToFill.size(); ++i )
    vectorToFill[ i ] = (T)i * stride;
}

template <typename T>
inline void permuteVector( VectorType<T>& vectorToPermute )
{
  for( size_t i = 0; i < vectorToPermute.size(); i++ )
  {
    size_t j = rand() % (i + 1);
    T temp = vectorToPermute[ j ];
    vectorToPermute[ j ] = vectorToPermute[ i ];
    vectorToPermute[ i ] = temp;
  }
}

inline unsigned int iPower2UpUnrolled(unsigned int v)
{
  --v;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return ++v;
}

inline void printIntVector( const IntVector& intVector ) {
  printf("{");
  for(IntVector::const_iterator it = intVector.begin(); it != intVector.end(); ++it) {
    printf(" %d ", *it);
  }
  printf("}\n");
}

}

#endif
