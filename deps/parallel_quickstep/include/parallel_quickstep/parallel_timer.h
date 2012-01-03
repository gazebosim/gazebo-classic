#ifndef PARALLEL_TIMER_H
#define PARALLEL_TIMER_H

#include <ode/timer.h>
#include "parallel_common.h"

class ParallelTimer
{
public:

  ParallelTimer( const char *description ) {
    dTimerNow( description );
  }

  virtual ~ParallelTimer( ) {
    dxGlobalSync();
  }

};

#endif
