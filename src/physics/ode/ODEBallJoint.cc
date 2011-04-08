/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: An ODE ball joint
 * Author: Nate Koenig
 * Date: k13 Oct 2009
 * SVN: $Id: ODEBallJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "gazebo_config.h"
#include "common/Console.hh"
#include "physics/ode/ODEBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEBallJoint::ODEBallJoint(dWorldID worldId)
    : BallJoint<ODEJoint>()
{
  this->jointId = dJointCreateBall(worldId, NULL);
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEBallJoint::~ODEBallJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
// Get the joints anchor point
common::Vector3 ODEBallJoint::GetAnchor(int index) const
{
  dVector3 result;
  // NATY
  // this->physics->LockMutex();
  dJointGetBallAnchor( jointId, result );
  // NATY
  // this->physics->UnlockMutex();

  return common::Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Set the joints anchor point
void ODEBallJoint::SetAnchor(int index, const common::Vector3 &anchor)
{
  // NATY
  // this->physics->LockMutex();
  dJointSetBallAnchor( jointId, anchor.x, anchor.y, anchor.z );
  // NATY 
  // this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODEBallJoint::SetDamping( int /*index*/, const double damping )
{
#ifdef INCLUDE_ODE_JOINT_DAMPING
  // NATY
  // this->physics->LockMutex();
  // ode does not yet support ball joint damping
  dJointSetDamping( this->jointId, damping);
  // NATY
  // this->physics->UnlockMutex();
#else
  // alternaitvely, apply explicit damping
  gzerr << "joint damping not implemented in ODE ball joint\n";
#endif
}

