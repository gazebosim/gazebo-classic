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
/* Desc: A slider or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */
#include <boost/bind.hpp>

#include "gazebo_config.h"
#include "common/Console.hh"
#include "common/XMLConfig.hh"

#include "physics/Body.hh"
#include "physics/ode/ODESliderJoint.hh"

using namespace gazebo;
using namespace physics;


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
void ODESliderJoint::Load(common::XMLConfigNode *node)
{
  SliderJoint<ODEJoint>::Load(node);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
common::Vector3 ODESliderJoint::GetAxis(int /*index*/) const
{
  dVector3 result;

  // NATY
  //this->physics->LockMutex();

  dJointGetSliderAxis( this->jointId, result );

  // NATY
  //this->physics->UnlockMutex();

  return common::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
common::Angle ODESliderJoint::GetAngle(int index) const
{
  // NATY
  //this->physics->LockMutex();
  
  common::Angle result = dJointGetSliderPosition( this->jointId );

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
void ODESliderJoint::SetAxis( int /*index*/, const common::Vector3 &axis )
{
  // NATY
  //this->physics->LockMutex();
  if (this->body1) 
    this->body1->SetEnabled(true);
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
