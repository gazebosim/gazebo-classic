#ifndef CUDA_UTILS_H
#define CUDA_UTILS_H

#include <cstdlib>
#include <cstdio>
#include <string.h>
#include <vector>

#include "cuda_common.h"

#define CUDA_CHECK_ERROR_BASE()          CUDA_CHECK_ERROR("")

#ifdef dDebug
//Check for CUDA error
#define CUDA_CHECK_ERROR(errorMessage)                                  \
  do									\
  {                                                                     \
    cudaError_t err = cudaGetLastError();                               \
    if(err != cudaSuccess)                                              \
    {                                                                   \
      fprintf(stderr,"Cuda error: %s in file '%s' in line %i : %s.\n",  \
              errorMessage, __FILE__, __LINE__, cudaGetErrorString( err)); \
      fprintf(stderr,"Press ENTER key to terminate the program\n");    \
      getchar();                                                        \
      exit(EXIT_FAILURE);                                               \
    }                                                                   \
    err = cudaThreadSynchronize();                                      \
    if(err != cudaSuccess)                                              \
    {                                                                   \
      fprintf(stderr,"Cuda error: %s in file '%s' in line %i : %s.\n",  \
              errorMessage, __FILE__, __LINE__, cudaGetErrorString( err)); \
      fprintf(stderr,"Press ENTER key to terminate the program\n");     \
      getchar();                                                        \
      exit(EXIT_FAILURE);                                               \
    }                                                                   \
  }                                                                     \
  while(0)

// Call and check function for CUDA error without synchronization
#define CUDA_SAFE_CALL_NO_SYNC(call)                                    \
  do                                                                    \
  {                                                                     \
    cudaError err = call;                                               \
    if(err != cudaSuccess)                                              \
    {                                                                   \
      fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",	\
              __FILE__, __LINE__, cudaGetErrorString( err) );           \
      fprintf(stderr,"Press ENTER key to terminate the program\n");     \
      getchar();                                                        \
      exit(EXIT_FAILURE);                                               \
    }                                                                   \
  }                                                                     \
  while(0)

// Call and check function for CUDA error with synchronization
#define CUDA_SAFE_CALL(call)                                            \
  do                                                                    \
  {                                                                     \
    CUDA_SAFE_CALL_NO_SYNC(call);                                       \
    cudaError err = cudaThreadSynchronize();                            \
    if(err != cudaSuccess)                                              \
    {                                                                   \
      fprintf(stderr,"Cuda errorSync in file '%s' in line %i : %s.\n",  \
              __FILE__, __LINE__, cudaGetErrorString( err) );           \
      fprintf(stderr,"Press ENTER key to terminate the program\n");     \
      getchar();                                                        \
      exit(EXIT_FAILURE);                                               \
    }                                                                   \
  } while (0)

#define CUDA_SAFE_CALL(call)                                            \
  do                                                                    \
  {                                                                     \
    CUDA_SAFE_CALL_NO_SYNC(call);                                       \
    cudaError err = cudaThreadSynchronize();                            \
    if(err != cudaSuccess)                                              \
    {                                                                   \
      fprintf(stderr,"Cuda errorSync in file '%s' in line %i : %s.\n",  \
              __FILE__, __LINE__, cudaGetErrorString( err) );           \
      fprintf(stderr,"Press ENTER key to terminate the program\n");     \
      getchar();                                                        \
      exit(EXIT_FAILURE);                                               \
    }                                                                   \
  } while (0)

#else // not DEBUG

#define CUDA_CHECK_ERROR(errorMessage)
#define CUDA_SAFE_CALL_NO_SYNC(call) call
#define CUDA_SAFE_CALL(call) call

#endif

inline void cudaDeviceInit(int dev = 0)
{
  int deviceCount;
  CUDA_SAFE_CALL_NO_SYNC(cudaGetDeviceCount(&deviceCount));
  if (deviceCount == 0) {
    fprintf(stderr, "CUTIL CUDA error: no devices supporting CUDA.\n");
    exit(-1);
  }
  if (dev < 0) dev = 0;
  if (dev > deviceCount-1) dev = deviceCount - 1;
  cudaDeviceProp deviceProp;
  CUDA_SAFE_CALL_NO_SYNC(cudaGetDeviceProperties(&deviceProp, dev));
  if (deviceProp.major < 1) {
    fprintf(stderr, "cutil error: GPU device does not support CUDA.\n");
    exit(-1);
  }
  CUDA_SAFE_CALL(cudaSetDevice(dev));
}

#endif
