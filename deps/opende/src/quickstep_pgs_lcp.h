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

#ifndef _ODE_QUICK_STEP_PGS_LCP_H_
#define _ODE_QUICK_STEP_PGS_LCP_H_

#include <ode/common.h>
#include "quickstep_util.h"
 
namespace ode {
    namespace quickstep{

// PGS_LCP was previously named SOR_LCP
void PGS_LCP (dxWorldProcessContext *context,
  const int m, const int nb, dRealMutablePtr J, dRealMutablePtr J_precon, dRealMutablePtr J_orig, dRealMutablePtr vnew, int *jb, dxBody * const *body,
  dRealPtr invMOI, dRealPtr MOI, dRealMutablePtr lambda, dRealMutablePtr lambda_erp,
  dRealMutablePtr caccel, dRealMutablePtr caccel_erp, dRealMutablePtr cforce,
  dRealMutablePtr rhs, dRealMutablePtr rhs_erp, dRealMutablePtr rhs_precon,
  dRealPtr lo, dRealPtr hi, dRealPtr cfm, const int *findex,
  dxQuickStepParameters *qs,
#ifdef USE_TPROW
  boost::threadpool::pool* row_threadpool,
#endif
  const dReal stepsize); 

size_t EstimatePGS_LCPMemoryRequirements(int m,int /*nb*/);

    } // namespace quickstep
} // namespace ode
#endif
