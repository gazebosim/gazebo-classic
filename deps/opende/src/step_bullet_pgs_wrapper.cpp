/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <ode/odeconfig.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"
#include "joints/hinge.h"

#include "gazebo/gazebo_config.h"
#include "step_bullet_pgs_wrapper.h"

#ifdef HAVE_BULLET
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wfloat-equal"
# pragma GCC diagnostic ignored "-Wshadow"
# pragma GCC diagnostic ignored "-Wunused-variable"
# include "LinearMath/btMatrixX.h"
# include "LinearMath/btAlignedObjectArray.h"
# include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
# include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
# pragma GCC diagnostic pop

//////////////////////////////////////////////////////////
void dSolveLCP_bullet_pgs(int _m, dReal *_A, dReal *_x, dReal *_b,
  dReal *_lo, dReal *_hi, int *findex)
{
  int i, j;
  btMatrixXu A(_m, _m);
  btVectorXu b(_m), x(_m), lo(_m), hi(_m);
  btAlignedObjectArray<int> limitDependency;
  // todo: pass from world file
  int numIterations = 1000;
  bool useSparsity = true;

  for(i=0; i<_m; ++i)
  {
    for(j=0; j<_m; ++j)
    {
      A.setElem(i, j, _A[i*_m+j]);
      if(i==j && dFabs(_A[i*_m+j]) < 1e-12)
        A.setElem(i, j, 1e-1);
    }
    b[i] = _b[i];
    x[i] = _x[i];
    lo[i] = _lo[i];
    hi[i] = _hi[i];
    limitDependency.push_back(findex[i]);
  }

  btSolveProjectedGaussSeidel* btPGS_solver = new btSolveProjectedGaussSeidel();
  btPGS_solver->solveMLCP(A, b, x, lo, hi, limitDependency, numIterations, useSparsity);

  // convert x to dReal format
  for(i=0; i<_m; i++)
    _x[i] = x[i];
}
#endif
