#ifndef PARALLEL_ODE_H
#define PARALLEL_ODE_H

#include <ode/objects.h>

#ifdef __cplusplus
extern "C" {
#endif

  ODE_API int dWorldParallelQuickStep(dWorldID w, dReal stepsize);

  ODE_API void dWorldSetParallelQuickStepSyncType(dWorldID, int syncType);

  ODE_API void dWorldSetParallelQuickStepReduceType(dWorldID, int reduceType);

#ifdef __cplusplus
}
#endif

#endif
