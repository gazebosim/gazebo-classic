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

#include <gazebo/ode/common.h>
#include "quickstep_util.h"

namespace ode {
    namespace quickstep{

// PGS_LCP was previously named SOR_LCP
void PGS_LCP (dxWorldProcessContext *context,
  const int m, const int nb, dRealMutablePtr J, dRealMutablePtr J_precon,
  dRealMutablePtr J_orig,
#ifdef PENETRATION_JVERROR_CORRECTION
  dRealMutablePtr vnew,
  const dReal stepsize,
#endif
  int *jb, dxBody * const *body,
  dRealPtr invMOI, dRealPtr MOI, dRealMutablePtr lambda,
  dRealMutablePtr lambda_erp,
  dRealMutablePtr caccel, dRealMutablePtr caccel_erp, dRealMutablePtr cforce,
  dRealMutablePtr rhs, dRealMutablePtr rhs_erp, dRealMutablePtr rhs_precon,
  dRealPtr lo, dRealPtr hi, dRealPtr cfm, const int *findex,
  dxQuickStepParameters *qs
#ifdef USE_TPROW
  , boost::threadpool::pool* row_threadpool
#endif
  );

/// \brief Compute the hi and lo bound for cone friction model to project onto
/// \param[in] lo_act The low bound for cone friction model to project onto
/// \param[in] hi_act The high bound for cone friction model to project onto
/// \param[in] lo_act_erp The erp-version low bound for cone friction model to project onto
/// \param[in] hi_act_erp The erp-version high bound for cone friction model to project onto
/// \param[in] jb         The list of consecutive body pair ids that form constraints
/// \param[in] J_orig     Unscaled Jacobian matrix J
/// \param[in] index      Index definition of the constraint type: -1 bilateral, -2 normal, >=0 friction
/// \param[in] startRow   Start row index
/// \param[in] nRows      Number of rows
/// \param[in] nb         Number of bodies
/// \param[in] body       Body array with kinematic information
/// \param[in] i          Current row index
/// \param[in] order      Constraint reorder
/// \param[in] findex     Constraint index, after reordering
/// \param[in] lo         Low bound for pyramid friction model to project onto
/// \param[in] hi         High bound for pyramid friction model to project onto
/// \param[in] lambda     Constraint force
/// \param[in] lambda_erp The erp-version of constraint force
///
void dxConeFrictionModel(dReal& lo_act, dReal& hi_act, dReal& lo_act_erp, dReal& hi_act_erp,
    int *jb, dRealPtr J_orig, int index, int constraint_index, int startRow,
    int nRows, const int nb, dxBody * const *body, int i, const IndexError *order,
    const int *findex, dRealPtr lo, dRealPtr hi, dRealMutablePtr lambda, dRealMutablePtr lambda_erp);

size_t EstimatePGS_LCPMemoryRequirements(int m,int /*nb*/);

    } // namespace quickstep
} // namespace ode
#endif
