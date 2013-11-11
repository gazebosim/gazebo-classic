#ifndef _PARALLEL_STEPPER_H_
#define _PARALLEL_STEPPER_H_

#include <ode/ode.h>

#include "util.h"

// TOOD: Add this struct to ode/src/objetcs.h

// Parallel-step parameters
struct dxParallelStepParameters {
  int syncType;
  int reduceType;
};

void dxParallelProcessIslands (dxWorld *world, dReal stepsize, dstepper_fn_t stepper);

void dxParallelQuickStepper ( //dxWorldProcessContext *shared_context,
                         dxWorldProcessContext *context,
                         dxWorld *world, dxBody * const *body, int nb,
                         dxJoint * const *_joint, int _nj, dReal stepsize);

bool dxReallocateParallelWorldProcessContext (dxWorld *world, dReal stepsize, dmemestimate_fn_t stepperestimate);

size_t dxEstimateParallelStepMemoryRequirements ( dxBody * const *body, int nb, dxJoint * const *_joint, int _nj);

template<typename T>
void compute_invM_JT (int m, const T* J, T* iMJ, int *jb,
                             dxBody * const *body, const T* invI);

template<typename T>
void compute_Adcfm_b (int m, T sor_w, T* J, const T* iMJ, int* jb, const T* cfm,
                      T* Adcfm, T* b);


#endif
