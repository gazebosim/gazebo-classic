#ifndef CUDA_TIMER_H
#define CUDA_TIMER_H

#include <cuda.h>
#include <ode/timer.h>

class CUDAODETimer
{
public:

  CUDAODETimer( const char *description ) {
    dTimerNow ( description );
  }

  ~CUDAODETimer( ) {
    cudaThreadSynchronize( );
  }
};

#endif
