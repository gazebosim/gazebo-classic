#ifndef OPENCL_UTILS_H
#define OPENCL_UTILS_H

#include <CL/cl.h>

#include <fstream>

//////////////////////////////////////////////////////////////////////////////////

#define oclCheckErrorEX(a, b, c) __oclCheckErrorEX(a, b, c, __FILE__ , __LINE__)
#define oclCheckError(a, b) oclCheckErrorEX(a, b, 0)

extern "C" cl_context oclGetGlobalContext( );
extern "C" cl_command_queue oclGetGlobalQueue( );

extern "C" void oclInit();
extern "C" void oclInitializeKernels(cl_context cxGPUContext, cl_command_queue cqCommandQueue);

extern "C" void oclShutdown(cl_context clContext, cl_command_queue clQueue);
extern "C" void oclShutdownKernels(void);

extern "C" cl_int oclGetPlatformID(cl_platform_id* clSelectedPlatformID);

extern "C" cl_device_id oclGetFirstDev(cl_context cxGPUContext);

extern "C" char* oclFindFilePath(const char* filename, const char* executablePath);
extern "C" char* oclLoadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength);
extern "C" const char* oclErrorString(cl_int error);

extern "C" void oclGetProgBinary( cl_program cpProgram, cl_device_id cdDevice, char** binary, size_t* length);
extern "C" void oclLogPtx(cl_program cpProgram, cl_device_id cdDevice, const char* cPtxFileName);
extern "C" void oclLogBuildInfo(cl_program cpProgram, cl_device_id cdDevice);

inline void __oclCheckErrorEX(cl_int iSample, cl_int iReference, void (*pCleanup)(int), const char* cFile, const int iLine)
{
  if (iReference != iSample)
  {
    iSample = (iSample == 0) ? -9999 : iSample;
    printf("\n !!! OpenCL Error # %i (%s) at line %i , in file %s !!!\n\n", iSample, oclErrorString(iSample), iLine, cFile);

    if (pCleanup != NULL) {
      pCleanup(iSample);
    } else {
      exit(iSample);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////

inline cl_context dxGetDeviceContext( ) { return oclGetGlobalContext(); }
inline cl_command_queue dxGetDeviceQueue( ) { return oclGetGlobalQueue(); }

inline void dxGlobalSync() { clFinish( dxGetDeviceQueue() ); }
inline void dxInitDevice() { oclInit(); oclInitializeKernels(dxGetDeviceContext(), dxGetDeviceQueue()); }
inline void dxShutdownDevice() { oclShutdown(dxGetDeviceContext(), dxGetDeviceQueue()); oclShutdownKernels(); }

//////////////////////////////////////////////////////////////////////////////////


#endif
