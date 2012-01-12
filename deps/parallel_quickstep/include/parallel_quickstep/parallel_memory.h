#ifndef PARALLEL_MEMORY_H
#define PARALLEL_MEMORY_H

#include "parallel_common.h"

namespace parallel_utils
{

namespace CopyTypes
{
  enum CopyType { COPY_SYNC, COPY_ASYNC, COPY_DEFAULT = COPY_ASYNC };
}
typedef CopyTypes::CopyType CopyType;

template<typename T>
static T offsetBuffer(T buffer, ArraySize offset) {
    return buffer + offset;
}

template <typename T, ParallelType PT> struct MemManager {

  typedef T* host_type;
  typedef T* device_type;
  typedef void** device_init_type;
  typedef int stream_type;
  typedef unsigned int mem_flags;

  inline static void allocateHost( void** ptr, size_t size, int flags ) {
    *ptr = malloc( size );
  }

  inline static void freeHost( void* ptr ) {
    free( ptr );
  }

  inline static void allocateDevice( void** ptr, size_t size ) {
    *ptr = NULL;
  }

  inline static void freeDevice( void* ptr ) {

  }

  inline static void copyToHost(void* dst, void* src, size_t size, CopyType copyType, stream_type stream ) {

  }

  inline static void copyToDevice(void* dst, void* src, size_t size, CopyType copyType, stream_type stream ) {

  }

};

#ifdef USE_CUDA

template <typename T>               struct MemManager<T,ParallelTypes::CUDA> {

  typedef T* host_type;
  typedef T* device_type;
  typedef void** device_init_type;
  typedef cudaStream_t stream_type;
  typedef unsigned int mem_flags;

  const static mem_flags defaultAllocFlags = cudaHostAllocPortable;

  inline static void allocateHost( void** ptr, size_t size, int flags ) {
    cudaHostAlloc(ptr, size, flags );
  }

  inline static void freeHost( void* ptr ) {
    cudaFreeHost( ptr );
  }

  inline static void allocateDevice( void** ptr, size_t size ) {
    cudaMalloc(ptr, size);
  }

  inline static void freeDevice( void* ptr ) {
    cudaFree( ptr );
  }

  inline static void copyToHost(void* dst, void * src, size_t size, CopyType copyType, stream_type stream ) {
    if( CopyTypes::COPY_SYNC == copyType )
      cudaMemcpy(dst, src, size, cudaMemcpyDeviceToHost);
    else
      cudaMemcpyAsync(dst, src, size, cudaMemcpyDeviceToHost, stream);
  }

  inline static void copyToDevice(void* dst, void * src, size_t size, CopyType copyType, stream_type stream ) {
    if( CopyTypes::COPY_SYNC == copyType )
      cudaMemcpy(dst, src, size, cudaMemcpyHostToDevice);
    else
      cudaMemcpyAsync(dst, src, size, cudaMemcpyHostToDevice, stream);
  }
};

#elif USE_OPENCL

// NOTE: If you don't have OpenCL 1.1, you're out of luck here =/
template<>
cl_mem offsetBuffer<cl_mem>(cl_mem buffer, ArraySize offset) {

  if( NULL == buffer || offset == 0 ) return buffer;

  size_t originalBufferSize;
  cl_int ciErrNum;
  ciErrNum = clGetMemObjectInfo( buffer,
                                 CL_MEM_SIZE,
                                 sizeof(size_t),
                                 &originalBufferSize,
                                 NULL );
  oclCheckError(ciErrNum, CL_SUCCESS);

  cl_mem_flags originalBufferMemflags;
  ciErrNum = clGetMemObjectInfo( buffer,
                                 CL_MEM_FLAGS,
                                 sizeof(cl_mem_flags),
                                 &originalBufferMemflags,
                                 NULL );
  oclCheckError(ciErrNum, CL_SUCCESS);

  cl_buffer_region subBufferRegion = { offset, originalBufferSize - offset };
  cl_mem subBuffer = clCreateSubBuffer( buffer,
                                        originalBufferMemflags,
                                        CL_BUFFER_CREATE_TYPE_REGION,
                                        &subBufferRegion,
                                        &ciErrNum );
  oclCheckError(ciErrNum, CL_SUCCESS);

  return subBuffer;
}

template <typename T>              struct MemManager<T, ParallelType::OpenCL> {

  typedef T* host_type;
  typedef cl_mem device_type;
  typedef cl_mem* device_init_type;
  typedef int stream_type;
  typedef cl_mem_flags mem_flags;

  inline static void allocateHost( void** ptr, size_t size, int flags ) {
    *ptr = malloc( size );
  }

  inline static void freeHost( void* ptr ) {
    if( ptr ) free( ptr );
  }

  inline static void allocateDevice( cl_mem* ptr, size_t size ) {
    cl_int ciErrNum;
    *ptr = clCreateBuffer( dxGetDeviceContext(), CL_MEM_READ_WRITE, size, NULL, &ciErrNum );
    oclCheckError(ciErrNum, CL_SUCCESS);
  }

  inline static void freeDevice( void* ptr ) {
    if( ptr ) {
      cl_int ciErrNum;
      ciErrNum = clReleaseMemObject( ((device_type)ptr) );
      oclCheckError(ciErrNum, CL_SUCCESS);
    }
  }

  inline static void copyToHost(void* dst, cl_mem src, size_t size, CopyType copyType, stream_type stream ) {
      cl_int ciErrNum;
      ciErrNum = clEnqueueReadBuffer(dxGetDeviceQueue(),
                                     src,
                                     (CopyTypes::COPY_SYNC == copyType ),
                                     0,
                                     size,
                                     dst,
                                     0,
                                     NULL,
                                     NULL);
      oclCheckError(ciErrNum, CL_SUCCESS);
  }

  inline static void copyToDevice(cl_mem dst, void* src, size_t size, CopyType copyType, stream_type stream ) {
      cl_int ciErrNum;
      ciErrNum = clEnqueueWriteBuffer(dxGetDeviceQueue(),
                                      dst,
                                      (CopyTypes::COPY_SYNC == copyType ),
                                      0,
                                      size,
                                      src,
                                      0,
                                      NULL,
                                      NULL);
      oclCheckError(ciErrNum, CL_SUCCESS);
  }
};

#endif

}

#endif

