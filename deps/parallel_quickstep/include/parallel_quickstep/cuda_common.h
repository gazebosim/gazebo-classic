#ifndef CUDA_COMMON_H
#define CUDA_COMMON_H

//////////////////////////////////////////////////////////////////////////////////

#define dxDeviceContext cudaStream_t
#define dxDeviceQueue cudaStream_t
#define dxParallelInfF CUDART_INF_F
#define dxParallelInfD CUDART_INF
#define dxOverload
#define dxDevice __device__
#define dxHost __host__
#define dxDeviceData
#define dxConstant __constant__
#define dxParams params
#define dxGlobal __global__
#define dxShared __shared__
#define dxSyncthreads() __syncthreads()

#define dxBlockIdx     blockIdx
#define dxBlockDim     blockDim
#define dxThreadIdx    threadIdx
#define dxGridDimX()   gridDim.x
#define dxBlockIdxX()  blockIdx.x
#define dxBlockDimX()  blockDim.x
#define dxThreadIdxX() threadIdx.x
#define dxGlobalOffsetX()    0
#define dxGlobalIdxX() (dxBlockIdxX() * dxBlockDimX() + dxThreadIdxX())

#define dxBlockIdxY()  blockIdx.y
#define dxBlockDimY()  blockDim.y
#define dxThreadIdxY() threadIdx.y
#define dxGlobalOffsetY()    0
#define dxGlobalIdxY() (dxBlockIdxY() * dxBlockDimY() + dxThreadIdxY())
#define dxExecKernel(numb, numt, kfunc, args) kfunc<<<numb, numt>>>args

//////////////////////////////////////////////////////////////////////////////////

// Note: For the CUDA runtime API, we have no notion of a context, these are just placeholders
static dxDeviceContext sDeviceContext = 0;
static dxDeviceQueue sDeviceQueue = 0;

//////////////////////////////////////////////////////////////////////////////////

inline dxDeviceContext dxGetDeviceContext( ) { return sDeviceContext; }
inline dxDeviceQueue dxGetDeviceQueue( ) { return sDeviceQueue; }
inline void dxGlobalSync() { cudaThreadSynchronize(); }
inline void dxInitDevice() { cudaFree(0); }
inline void dxShutdownDevice() { cudaThreadExit(); }

#endif
