#ifndef OPENCL_COMMON_H
#define OPENCL_COMMON_H

//////////////////////////////////////////////////////////////////////////////////

#define dxDeviceContext cl_context
#define dxDeviceQueue cl_command_queue
#define dxParallelInfF CL_HUGEVAL_F
#define dxParallelInfD CL_HUGEVAL
#define dxOverload __OVERLOADABLE__
#define dxDevice
#define dxHost
#define dxDeviceData __global
#define dxConstant __constant
#define dxParams params
#define dxMul24(a, b) mul24(a, b)
#define dxGlobal __kernel
#define dxShared __local
#define dxSyncthreads() barrier(CLK_LOCAL_MEM_FENCE)

#define dxGridDimX() get_num_groups(0)
#define dxBlockIdxX() get_group_id(0)
#define dxBlockDimX() get_local_size(0)
#define dxThreadIdxX() get_local_id(0)
#define dxGlobalOffsetX() get_global_offset(0)
#define dxGlobalIdxX() get_global_id(0)

#define dxBlockIdxY() get_group_id(1)
#define dxBlockDimY() get_local_size(1)
#define dxThreadIdxY() get_local_id(1)
#define dxGlobalOffsetY() get_global_offset(1)
#define dxGlobalIdxY() get_global_id(1)

#define dxExecKernel(numb, numt, kfunc, args)
//kfunc<<<numb, numt>>>args

//////////////////////////////////////////////////////////////////////////////////

#endif
