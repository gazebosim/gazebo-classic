#ifndef OPENMP_COMMON_H
#define OPENMP_COMMON_H

//////////////////////////////////////////////////////////////////////////////////

#define dxDeviceContext int
#define dxDeviceQueue int
#define dxParallelInfF std::numeric_limits<float>::infinity()
#define dxParallelInfD std::numeric_limits<double>::infinity()
#define dxOverload
#define dxConstant
#define dxDevice
#define dxDeviceData
#define dxHost
#define dxMul24(a, b) ((a)*(b))
#define dxParams
#define dxGlobal
#define dxShared static
#define dxSyncthreads()
#define dxBlockIdx s_blockIdx
#define dxBlockDim s_blockDim
#define dxThreadIdx s_threadIdx

//////////////////////////////////////////////////////////////////////////////////

// Note: For OpenMP, we have no notion of a context, these are just placeholders
static dxDeviceContext sCpuContext = 0;
static dxDeviceQueue sDeviceQueue = 0;

//////////////////////////////////////////////////////////////////////////////////

inline dxDeviceContext dxGetDeviceContext( ) { return sCpuContext; }
inline dxDeviceQueue dxGetDeviceQueue( ) { return sDeviceQueue; }
inline void dxGlobalSync() { }
inline void dxInitDevice() { }
inline void dxShutdownDevice() { }

#endif
