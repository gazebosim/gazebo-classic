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
/* Desc: A screw or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODEScrewJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "gazebo_config.h"
#include "GazeboMessage.hh"
#include "Body.hh"
#include "XMLConfig.hh"
#include "ODEScrewJoint.hh"
#include <boost/signal.hpp>
#include <boost/bind.hpp>

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEScrewJoint::ODEScrewJoint( dWorldID worldId )
    : ScrewJoint<ODEJoint>()
{
  Param::Begin(&this->parameters);
  this->threadPitchP = new ParamT<double>("threadPitch", 1.0, 0);
  Param::End();
#if ODE_SCREW_JOINT
  this->jointId = dJointCreateScrew( worldId, NULL );
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEScrewJoint::~ODEScrewJoint()
{
  delete this->threadPitchP;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void ODEScrewJoint::Load(XMLConfigNode *node)
{
  ScrewJoint<ODEJoint>::Load(node);
  this->threadPitchP->Load(node);
  this->SetThreadPitch(0, this->threadPitchP->GetValue());
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 ODEScrewJoint::GetAxis(int /*index*/) const
{
  dVector3 result;
  this->physics->LockMutex();
#if ODE_SCREW_JOINT
  dJointGetScrewAxis( this->jointId, result );
#endif
  this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
Angle ODEScrewJoint::GetAngle(int index) const
{
  this->physics->LockMutex();
#if ODE_SCREW_JOINT
  Angle result = dJointGetScrewPosition( this->jointId );
#else
  Angle result(0.0);
#endif
  this->physics->UnlockMutex();
  
  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double ODEScrewJoint::GetVelocity(int /*index*/) const
{
  this->physics->LockMutex();
#if ODE_SCREW_JOINT
  double result = dJointGetScrewPositionRate( this->jointId );
#else
  double result = 0.0;
#endif
  this->physics->UnlockMutex();

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODEScrewJoint::SetVelocity(int /*index*/, double angle)
{
  this->SetParam(dParamVel, angle);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of motion
void ODEScrewJoint::SetAxis( int /*index*/, const Vector3 &axis )
{
  this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

#if ODE_SCREW_JOINT
  dJointSetScrewAxis( this->jointId, axis.x, axis.y, axis.z );
#endif
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODEScrewJoint::SetDamping( int /*index*/, const double damping )
{
  this->damping_coefficient = damping;
#ifdef INCLUDE_ODE_JOINT_DAMPING
  this->physics->LockMutex();
  dJointSetDamping( this->jointId, this->damping_coefficient);
  this->physics->UnlockMutex();
#else
  // alternaitvely, apply explicit damping
  this->ConnectJointUpdateSignal(boost::bind(&ODEScrewJoint::ApplyDamping,this));
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Set thread pitch
void ODEScrewJoint::SetThreadPitch( int /*index*/, const double thread_pitch )
{
  this->physics->LockMutex();
#if ODE_SCREW_JOINT
  //std::cout << "\n\n\n\nsetting threadPitch: " << thread_pitch << "\n\n\n\n\n";
  dJointSetScrewThreadPitch( this->jointId, thread_pitch);
#endif
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// callback to apply joint damping force
void ODEScrewJoint::ApplyDamping()
{
  double damping_force = this->damping_coefficient * this->GetVelocity(0);
  this->SetForce(0,damping_force);
}

//////////////////////////////////////////////////////////////////////////////
// Set the screw force
void ODEScrewJoint::SetForce(int /*index*/, double force)
{
  this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

#if ODE_SCREW_JOINT
  //dJointAddScrewForce(this->jointId, force);
  dJointAddScrewTorque(this->jointId, force);
#endif
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter
void ODEScrewJoint::SetParam( int parameter, double value )
{
  this->physics->LockMutex();
  ODEJoint::SetParam(parameter, value);
#if ODE_SCREW_JOINT
  dJointSetScrewParam( this->jointId, parameter, value );
#endif
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Get the _parameter
double ODEScrewJoint::GetParam( int parameter ) const
{
  this->physics->LockMutex();
#if ODE_SCREW_JOINT
  double result = dJointGetScrewParam( this->jointId, parameter );
#else
  double result = 0.0;
#endif
  this->physics->UnlockMutex();

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODEScrewJoint::SetMaxForce(int index, double t) 
{
  this->SetParam(dParamFMax, t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODEScrewJoint::GetMaxForce(int index)
{
  this->GetParam(dParamFMax);
}
