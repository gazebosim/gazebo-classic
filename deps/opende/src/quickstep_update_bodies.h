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

#ifndef _ODE_QUICK_STEP_UPDATE_BODIES_H_
#define _ODE_QUICK_STEP_UPDATE_BODIES_H_

#include <ode/common.h>
#include "quickstep_util.h"

namespace ode {
    namespace quickstep{
        // update velocity and position of all bodies
        void dxUpdateBodies(
#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
                dxWorldProcessContext *context, const int *findex,
                dRealMutablePtr const cforce, dxQuickStepParameters *qs,
#endif
                const int m, const int mfb, dxBody *const *body, int nb,
                dJointWithInfo1 *const jointiinfos, int nj, const dReal stepsize,
                dRealMutablePtr const lambda, dRealMutablePtr const  caccel,
                dRealMutablePtr const caccel_erp, dRealPtr Jcopy,  dRealPtr invMOI);

        size_t dxUpdateBodiesMemoryRequirements(int m, int nb);
    } // namespace quickstep
} // namespace ode
#endif
