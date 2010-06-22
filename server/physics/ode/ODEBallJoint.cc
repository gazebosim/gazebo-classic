/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: An ODE ball joint
 * Author: Nate Koenig
 * Date: k13 Oct 2009
 * SVN: $Id: ODEBallJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "gazebo_config.h"
#include "GazeboMessage.hh"
#include "ODEBallJoint.hh"

using namespace gazebo;

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
Vector3 ODEBallJoint::GetAnchor(int index) const
{
  dVector3 result;
  this->physics->LockMutex();
  dJointGetBallAnchor( jointId, result );
  this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Set the joints anchor point
void ODEBallJoint::SetAnchor(int index, const Vector3 &anchor)
{
  this->physics->LockMutex();
  dJointSetBallAnchor( jointId, anchor.x, anchor.y, anchor.z );
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODEBallJoint::SetDamping( int /*index*/, const double damping )
{
#ifdef INCLUDE_ODE_JOINT_DAMPING
  this->physics->LockMutex();
  // ode does not yet support ball joint damping
  dJointSetDamping( this->jointId, damping);
  this->physics->UnlockMutex();
#else
  // alternaitvely, apply explicit damping
  gzerr(0) << "joint damping not implemented in ODE ball joint\n";
#endif
}

