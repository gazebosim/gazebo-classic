/*************************************************************************
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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
#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include <ode/misc.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"
#include <sys/time.h>
#include "quickstep_util.h"
#include "quickstep_cg_lcp.h"
using namespace ode;

//***************************************************************************
// conjugate gradient method with jacobi preconditioner
// THIS IS EXPERIMENTAL CODE that doesn't work too well, so it is ifdefed out.
//
// adding CFM seems to be critically important to this method.

#ifdef USE_CG_LCP

void quickstep::CG_LCP (dxWorldProcessContext *context,
  int m, int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
  dRealPtr invMOI, dRealMutablePtr lambda, dRealMutablePtr cforce, dRealMutablePtr rhs,
  dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
  dxQuickStepParameters *qs)
{
  const int num_iterations = qs->num_iterations;

  // precompute iMJ = inv(M)*J'
  dReal *iMJ = context->AllocateArray<dReal> (m*12);
  compute_invM_JT (m,J,iMJ,jb,body,invMOI);

  dReal last_rho = 0;
  dReal *r = context->AllocateArray<dReal> (m);
  dReal *z = context->AllocateArray<dReal> (m);
  dReal *p = context->AllocateArray<dReal> (m);
  dReal *q = context->AllocateArray<dReal> (m);

  // precompute 1 / diagonals of A
  dReal *Ad = context->AllocateArray<dReal> (m);
  dRealPtr iMJ_ptr = iMJ;
  dRealPtr J_ptr = J;
  for (int i=0; i<m; i++) {
    dReal sum = 0;
    for (int j=0; j<6; j++) sum += iMJ_ptr[j] * J_ptr[j];
    if (jb[i*2+1] >= 0) {
      for (int j=6; j<12; j++) sum += iMJ_ptr[j] * J_ptr[j];
    }
    iMJ_ptr += 12;
    J_ptr += 12;
    Ad[i] = REAL(1.0) / (sum + cfm[i]);
  }

  if (qs->warm_start > 0)
  {
    // warm start
    // compute residual r = rhs - A*lambda
    multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,cforce,lambda,r);
    for (int k=0; k<m; k++) r[k] = rhs[k] - r[k];
  }
  else
  {
    dSetZero (lambda,m);
    memcpy (r,rhs,m*sizeof(dReal));    // residual r = rhs - A*lambda
  }

  for (int iteration=0; iteration < num_iterations; iteration++) {
    for (int i=0; i<m; i++) z[i] = r[i]*Ad[i];  // z = inv(M)*r
    dReal rho = quickstep::dot_n (m,r,z);    // rho = r'*z

    // @@@
    // we must check for convergence, otherwise rho will go to 0 if
    // we get an exact solution, which will introduce NaNs into the equations.
    if (rho < 1e-10) {
      printf ("CG returned at iteration %d\n",iteration);
      break;
    }

    if (iteration==0) {
      memcpy (p,z,m*sizeof(dReal));  // p = z
    }
    else {
      quickstep::scaled_add (m,p,z,p,rho/last_rho);  // p = z + (rho/last_rho)*p
    }

    // compute q = (J*inv(M)*J')*p
    multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,cforce,p,q);

    dReal alpha = rho/quickstep::dot_n (m,p,q);    // alpha = rho/(p'*q)
    quickstep::scaled_add (m,lambda,lambda,p,alpha);    // lambda = lambda + alpha*p
    quickstep::scaled_add (m,r,r,q,-alpha);      // r = r - alpha*q
    last_rho = rho;
  }

  // compute cforce = inv(M)*J'*lambda
  multiply_invM_JT (m,nb,iMJ,jb,lambda,cforce);

#if 0
  // measure solution error
  multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,cforce,lambda,r);
  dReal error = 0;
  for (int i=0; i<m; i++) error += dFabs(r[i] - rhs[i]);
  printf ("lambda error = %10.6e\n",error);
#endif
}
 
size_t quickstep::EstimateGR_LCPMemoryRequirements(int m)
{
  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for iMJ
  res += 5 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for r, z, p, q, Ad
  return res;
}
#endif