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
#include "step_bullet_lemke_wrapper.h"

#ifdef HAVE_BULLET
#ifdef LIBBULLET_VERSION_GT_282
#include "step_bullet_inc.h"

//////////////////////////////////////////////////////////
void dSolveLCP_bullet_lemke(int _m, dReal *_A, dReal *_x, dReal *_b,
  dReal *_lo, dReal *_hi)
{
  int i, j, rindex, cindex;
  btMatrixXu A(_m, _m);
  btVectorXu b(_m), x(_m), lo(_m), hi(_m);
  btAlignedObjectArray<int> limitDependency;
  // TODO: pass from world file
  int numIterations = 1000;
  bool useSparsity = true;
  rindex = 0;
  for (i=0; i<_m; ++i)
  {
    cindex = 0;
    for (j=0; j<_m+1; ++j)
    {
      if (j<_m)
        A.setElem(rindex, cindex, _A[i*(_m+1)+j]);
      if(i==j && dFabs(A(i, j)) < 1e-12)
        A.setElem(rindex, cindex, 1e-5);
      cindex++;
    }
    rindex++;
    b[i] = _b[i];
    x[i] = _x[i];
    lo[i] = _lo[i];
    hi[i] = _hi[i];
  }

  btLemkeSolver* btlemke_solver = new btLemkeSolver();
  btlemke_solver->solveMLCP(A, b, x, lo, hi,
    limitDependency, numIterations, useSparsity);

  // convert x to dReal format
  for(i=0; i<_m; i++)
    _x[i] = x[i];
}
#endif
#endif
