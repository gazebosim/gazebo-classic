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

#include "physics/Link.hh"
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
void ODESliderJoint::Load( sdf::ElementPtr &_sdf )
{
  SliderJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
math::Vector3 ODESliderJoint::GetAxis(int /*_index*/) const
{
  dVector3 result;
  dJointGetSliderAxis( this->jointId, result );
  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
math::Angle ODESliderJoint::GetAngle(int /*_index*/) const
{
  math::Angle result = dJointGetSliderPosition( this->jointId );
  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double ODESliderJoint::GetVelocity(int /*index*/) const
{
  double result = dJointGetSliderPositionRate( this->jointId );
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
void ODESliderJoint::SetAxis( int /*index*/, const math::Vector3 &axis )
{
  if (this->childLink) 
    this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  dJointSetSliderAxis( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODESliderJoint::SetDamping( int /*index*/, const double damping )
{
  this->damping_coefficient = damping;
#ifdef INCLUDE_ODE_JOINT_DAMPING
  // ode does not yet support slider joint damping
  dJointSetDamping( this->jointId, this->damping_coefficient);
#else
  // alternaitvely, apply explicit damping
  this->ConnectJointUpdate(boost::bind(&ODESliderJoint::ApplyDamping,this));
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
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  dJointAddSliderForce(this->jointId, force);
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter
void ODESliderJoint::SetParam( int parameter, double value )
{
  ODEJoint::SetParam(parameter, value);
  dJointSetSliderParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Get the _parameter
double ODESliderJoint::GetParam( int parameter ) const
{
  double result = dJointGetSliderParam( this->jointId, parameter );
  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODESliderJoint::SetMaxForce(int /*_index*/, double _t) 
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODESliderJoint::GetMaxForce(int /*_index*/)
{
  return this->GetParam(dParamFMax);
}
