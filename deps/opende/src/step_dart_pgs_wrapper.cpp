/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/ode/odeconfig.h>
#include <gazebo/ode/odemath.h>
#include <gazebo/ode/rotation.h>
#include <gazebo/ode/timer.h>
#include <gazebo/ode/error.h>
#include <gazebo/ode/matrix.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"
#include "joints/hinge.h"

#include "gazebo/gazebo_config.h"

#ifdef HAVE_DART
#include "dart/constraint/PGSLCPSolver.h"
#include "step_dart_pgs_wrapper.h"

void dSolveLCP_dart_pgs(int m, int mskip, dReal *A, dReal *x, dReal *b,
        int nub, dReal *lo, dReal *hi, int *findex)
{
    // For the case m = 0, return true;
    if(m>0)
    {
      dart::constraint::PGSOption option;
      option.setDefault();
      dart::constraint::solvePGS(m, mskip, nub, A, x, b, lo, hi, findex, &option);
    }
}
#endif
