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
/* Desc: A slider or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODESliderJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "gazebo_config.h"
#include "GazeboMessage.hh"
#include "Body.hh"
#include "XMLConfig.hh"
#include "ODESliderJoint.hh"
#include <boost/signal.hpp>
#include <boost/bind.hpp>

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODESliderJoint::ODESliderJoint( dWorldID worldId )
    : SliderJoint<ODEJoint>()
{
  this->jointId = dJointCreateSlider( worldId, NULL );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODESliderJoint::~ODESliderJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void ODESliderJoint::Load(XMLConfigNode *node)
{
  SliderJoint<ODEJoint>::Load(node);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 ODESliderJoint::GetAxis(int /*index*/) const
{
  dVector3 result;

  // NATY
  //this->physics->LockMutex();

  dJointGetSliderAxis( this->jointId, result );

  // NATY
  //this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
Angle ODESliderJoint::GetAngle(int index) const
{
  // NATY
  //this->physics->LockMutex();
  
  Angle result = dJointGetSliderPosition( this->jointId );

  // NATY
  //this->physics->UnlockMutex();
  
  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double ODESliderJoint::GetVelocity(int /*index*/) const
{
  // NATY
  //this->physics->LockMutex();
  double result = dJointGetSliderPositionRate( this->jointId );
  // NATY
  //this->physics->UnlockMutex();

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODESliderJoint::SetVelocity(int /*index*/, double angle)
{
  this->SetParam(dParamVel, angle);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of motion
void ODESliderJoint::SetAxis( int /*index*/, const Vector3 &axis )
{
  // NATY
  //this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

  dJointSetSliderAxis( this->jointId, axis.x, axis.y, axis.z );
  // NATY
  //this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODESliderJoint::SetDamping( int /*index*/, const double damping )
{
  this->damping_coefficient = damping;
#ifdef INCLUDE_ODE_JOINT_DAMPING
  // NATY
  //this->physics->LockMutex();
  // ode does not yet support slider joint damping
  dJointSetDamping( this->jointId, this->damping_coefficient);
  // NATY
  //this->physics->UnlockMutex();
#else
  // alternaitvely, apply explicit damping
  this->ConnectJointUpdateSignal(boost::bind(&ODESliderJoint::ApplyDamping,this));
#endif
}

//////////////////////////////////////////////////////////////////////////////
// callback to apply joint damping force
void ODESliderJoint::ApplyDamping()
{
  double damping_force = this->damping_coefficient * this->GetVelocity(0);
  this->SetForce(0,damping_force);
}

//////////////////////////////////////////////////////////////////////////////
// Set the slider force
void ODESliderJoint::SetForce(int /*index*/, double force)
{
  // NATY
  //this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

  dJointAddSliderForce(this->jointId, force);
  // NATY
  //this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter
void ODESliderJoint::SetParam( int parameter, double value )
{
  // NATY
  //this->physics->LockMutex();
  ODEJoint::SetParam(parameter, value);
  dJointSetSliderParam( this->jointId, parameter, value );
  // NATY
  // this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Get the _parameter
double ODESliderJoint::GetParam( int parameter ) const
{
  // NATY
  //this->physics->LockMutex();
  double result = dJointGetSliderParam( this->jointId, parameter );
  // NATY
  //this->physics->UnlockMutex();

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODESliderJoint::SetMaxForce(int index, double t) 
{
  this->SetParam(dParamFMax, t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODESliderJoint::GetMaxForce(int index)
{
  this->GetParam(dParamFMax);
}
