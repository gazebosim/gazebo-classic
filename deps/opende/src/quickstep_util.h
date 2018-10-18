/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_QUICK_STEP_UTIL_H_
#define _ODE_QUICK_STEP_UTIL_H_

#include <gazebo/ode/common.h>
#include "gazebo/gazebo_config.h"

#ifdef ODE_SSE
#include <xmmintrin.h>
#define Kf(x) _mm_set_pd((x),(x))
#endif


#undef REPORT_THREAD_TIMING
#undef USE_TPROW
#undef TIMING
#undef DEBUG_CONVERGENCE_TOLERANCE
#undef SHOW_CONVERGENCE
#define SMOOTH_LAMBDA
#undef USE_1NORM
#undef DEBUG_INERTIA_PROPAGATION
//#define LOCAL_STEPPING  // not yet implemented
//#define PENETRATION_JVERROR_CORRECTION
//#define POST_UPDATE_CONSTRAINT_VIOLATION_CORRECTION

#undef CHECK_VELOCITY_OBEYS_CONSTRAINT

#ifndef TIMING
#ifdef HDF5_INSTRUMENT
#define DUMP
#include <gazebo/ode/h5dump.h>
#define DATA_FILE  "ode_frames.hdf5"
#endif  // instrument
#endif  // timing

#ifdef USE_TPROW
// added for threading per constraint rows
#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include "gazebo/ode/odeinit.h"
#endif

typedef const dReal *dRealPtr;
typedef dReal *dRealMutablePtr;

//***************************************************************************
// configuration

// for the PGS and CG methods:
// warm starting:
// this definitely help for motor-driven joints.
// unfortunately it appears to hurt with high-friction contacts
// using the PGS method. use with care

// for the PGS method:
// uncomment the following line to determine a new constraint-solving
// order for each iteration. however, the qsort per iteration is expensive,
// and the optimal order is somewhat problem dependent.
// @@@ try the leaf->root ordering.

// #define REORDER_CONSTRAINTS 1

// for the PGS method:
// uncomment the following line to randomly reorder constraint rows
// during the solution. depending on the situation, this can help a lot
// or hardly at all, but it doesn't seem to hurt.

// #define RANDOMLY_REORDER_CONSTRAINTS 1
#undef LOCK_WHILE_RANDOMLY_REORDER_CONSTRAINTS

//***************************************************************************
// testing stuff

#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif

#ifdef DUMP
#define IFDUMP(x) x
#else
#define IFDUMP(x) ((void)0)
#endif
// ****************************************************************
// ******************* Struct Definition **************************
// ****************************************************************
struct dJointWithInfo1
{
  dxJoint *joint;
  dxJoint::Info1 info;
};

struct IndexError {
#ifdef REORDER_CONSTRAINTS
  dReal error;    // error to sort on
  int findex;
#endif
  int index;    // row index
};

// structure for passing variable pointers in PGS_LCP
struct dxPGSLCPParameters {
    int thread_id;
    IndexError* order;
    dxBody* const* body;
    boost::recursive_mutex* mutex;
    bool inline_position_correction;
    bool position_correction_thread;
    dxQuickStepParameters *qs;
    int nStart;   // 0
    int nChunkSize;
    int m; // m
    int nb;
#ifdef PENETRATION_JVERROR_CORRECTION
    dReal stepsize;
    dRealMutablePtr vnew;
#endif
    int* jb;
    const int* findex;
    bool skip_friction;
    dRealPtr hi;
    dRealPtr lo;
    dRealPtr invMOI;
    dRealPtr MOI;
    dRealPtr Ad;
    dRealPtr Adcfm;
    dRealPtr Adcfm_precon;
    dRealPtr J;
    dRealPtr iMJ;
    dRealPtr rhs_precon ;
    dRealPtr J_precon ;
    dRealPtr J_orig ;
    dRealMutablePtr cforce ;

    dRealPtr rhs;
    dRealMutablePtr caccel;
    dRealMutablePtr lambda;

    /// Only used if THREAD_POSITION_CORRECTION is not active,
    /// in that case, compute both updates in the same
    /// ComputeRows update.
    dRealPtr rhs_erp;
    dRealMutablePtr caccel_erp;
    dRealMutablePtr lambda_erp;

#ifdef REORDER_CONSTRAINTS
    dRealMutablePtr last_lambda ;
    dRealMutablePtr last_lambda_erp;
#endif
};
// ****************************************************************
// ******************* Util Functions *****************************
// ****************************************************************

//****************************************************************************
// special matrix multipliers

namespace ode {
    namespace quickstep{

// multiply block of B matrix (q x 6) with 12 dReal per row with C vektor (q)
void Multiply1_12q1 (dReal *A, const dReal *B, const dReal *C, int q);

// compute iMJ = inv(M)*J'
void compute_invM_JT (int m, dRealPtr J, dRealMutablePtr iMJ, int *jb,
  dxBody * const *body, dRealPtr invMOI);

// compute out = inv(M)*J'*in.
void multiply_invM_JT (int m, int nb, dRealMutablePtr iMJ, int *jb,
  dRealPtr in, dRealMutablePtr out);

// compute out = J*in.
void multiply_J (int m, dRealPtr J, int *jb,
  dRealPtr in, dRealMutablePtr out);

#ifdef USE_CG_LCP
// compute out = (J*inv(M)*J' + cfm)*in.
void multiply_J_invM_JT (int m, int nb, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
  dRealPtr cfm, dRealMutablePtr z, dRealMutablePtr in, dRealMutablePtr out);
#endif

// dot product of two vector x, y with length of n
inline dReal dot_n (int n, dRealPtr x, dRealPtr y)
{
  dReal sum=0;
  for (int i=0; i<n; i++) sum += x[i]*y[i];
  return sum;
}

// x = y + z*alpha
inline void scaled_add (int n, dRealMutablePtr x, dRealPtr y, dRealPtr z, dReal alpha)
{
  for (int i=0; i<n; i++) x[i] = y[i] + z[i]*alpha;
}

// dot product of two vector a and b with length 6
// define ODE_SSE to enable SSE, which is used to speed up
// vector math operations with gcc compiler
// macro SSE is renamed to ODE_SSE due to conflict with Eigen3 in DART
inline dReal dot6(dRealPtr a, dRealPtr b)
{
#ifdef ODE_SSE
  __m128d d = _mm_load_pd(a+0) * _mm_load_pd(b+0) + _mm_load_pd(a+2) * _mm_load_pd(b+2) + _mm_load_pd(a+4) * _mm_load_pd(b+4);
  double r[2];
  _mm_store_pd(r, d);
  return r[0] + r[1];
#else
  return a[0] * b[0] +
         a[1] * b[1] +
         a[2] * b[2] +
         a[3] * b[3] +
         a[4] * b[4] +
         a[5] * b[5];
#endif
}

// a = a + delta * b, vector a and b with length 6
inline void sum6(dRealMutablePtr a, dReal delta, dRealPtr b)
{
#ifdef ODE_SSE
  __m128d __delta = Kf(delta);
  _mm_store_pd(a + 0, _mm_load_pd(a + 0) + __delta * _mm_load_pd(b + 0));
  _mm_store_pd(a + 2, _mm_load_pd(a + 2) + __delta * _mm_load_pd(b + 2));
  _mm_store_pd(a + 4, _mm_load_pd(a + 4) + __delta * _mm_load_pd(b + 4));
#else
  a[0] += delta * b[0];
  a[1] += delta * b[1];
  a[2] += delta * b[2];
  a[3] += delta * b[3];
  a[4] += delta * b[4];
  a[5] += delta * b[5];
#endif
}

// compare the index error when REORDER_CONSTRAINTS is defined
int compare_index_error (const void *a, const void *b);

// Modifying inertia along constrained axes without modifying dynamics.
void DYNAMIC_INERTIA(const int infom, const dxJoint::Info2 &Jinfo, const int b1, const int b2,
                            const dJointWithInfo1 *jicurr,
                            dRealMutablePtr invMOI, dRealMutablePtr MOI);
    } // namespace quickstep
} // namespace ode
#endif
