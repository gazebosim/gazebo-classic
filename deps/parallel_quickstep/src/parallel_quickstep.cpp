#include <parallel_quickstep.h>
#include <parallel_stepper.h>

#include "quickstep.h"
#include "util.h"

/** @todo Add the cs object to dxWorld */
static dxParallelStepParameters cs;

int dWorldParallelQuickStep (dWorldID w, dReal stepsize)
{
  dUASSERT (w,"bad world argument");
  dUASSERT (stepsize > 0,"stepsize must be > 0");

  bool result = false;

  if( dxReallocateParallelWorldProcessContext (w, stepsize, &dxEstimateParallelStepMemoryRequirements) ) {
    dxParallelProcessIslands (w, stepsize, &dxParallelQuickStepper);
    result = true;
  }

  return result;
}

void dWorldSetParallelQuickStepSyncType(dWorldID w, int syncType)
{
  dAASSERT(w);
  //w->cs.syncType = syncType;
  cs.syncType = syncType;
}

void dWorldSetParallelQuickStepReduceType(dWorldID w, int reduceType)
{
  dAASSERT(w);
  //w->cs.reduceType = reduceType;
  cs.reduceType = reduceType;
}



