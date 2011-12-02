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
/* Desc: A ODEHingeJoint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */

#include <boost/bind.hpp>

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/ode/ODEHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEHingeJoint::ODEHingeJoint( dWorldID worldId )
    : HingeJoint<ODEJoint>()
{
  this->jointId = dJointCreateHinge(worldId, NULL);
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEHingeJoint::~ODEHingeJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load a hinge joint
void ODEHingeJoint::Load( sdf::ElementPtr &_sdf )
{
  HingeJoint<ODEJoint>::Load(_sdf);

  this->SetParam(dParamFMax, 0);
  this->SetForce(0, 0);
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
math::Vector3 ODEHingeJoint::GetAnchor(int /*index*/) const
{
  dVector3 result;

  dJointGetHingeAnchor( this->jointId, result );

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void ODEHingeJoint::SetAnchor( int /*index*/, const math::Vector3 &anchor )
{
  if (this->childLink) 
    this->childLink->SetEnabled(true);
  if (this->parentLink) 
    this->parentLink->SetEnabled(true);

  dJointSetHingeAnchor( this->jointId, anchor.x, anchor.y, anchor.z );
}


//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
math::Vector3 ODEHingeJoint::GetGlobalAxis(int /*_index*/) const
{
    dVector3 result;
    dJointGetHingeAxis( this->jointId, result );
    return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of rotation
void ODEHingeJoint::SetAxis( int /*index*/, const math::Vector3 &axis )
{
  if (this->childLink) 
    this->childLink->SetEnabled(true);
  if (this->parentLink) 
    this->parentLink->SetEnabled(true);

  dJointSetHingeAxis( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping, either through ODE or callback here
void ODEHingeJoint::SetDamping( int /*index*/, const double damping )
{
  this->damping_coefficient = damping;
  dJointSetDamping( this->jointId, this->damping_coefficient);
}

//////////////////////////////////////////////////////////////////////////////
// callback to apply joint damping force
void ODEHingeJoint::ApplyDamping()
{
  double damping_force = this->damping_coefficient * this->GetVelocity(0);
  this->SetForce(0,damping_force);
}

//////////////////////////////////////////////////////////////////////////////
// Get the angle of rotation
math::Angle ODEHingeJoint::GetAngleImpl(int /*index*/) const
{
  math::Angle result = dJointGetHingeAngle( this->jointId );

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the rotation rate
double ODEHingeJoint::GetVelocity(int /*index*/) const
{
  double result = dJointGetHingeAngleRate( this->jointId );

  return result;
} 

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODEHingeJoint::SetVelocity(int /*index*/, double angle)
{
  this->SetParam(dParamVel,angle);
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODEHingeJoint::SetMaxForce(int /*index*/, double t)
{
  return this->SetParam(dParamFMax, t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODEHingeJoint::GetMaxForce(int /*index*/)
{
  return this->GetParam(dParamFMax);
}

//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void ODEHingeJoint::SetForce(int /*index*/, double torque)
{
  if (this->childLink) 
    this->childLink->SetEnabled(true);
  if (this->parentLink) 
    this->parentLink->SetEnabled(true);
  dJointAddHingeTorque( this->jointId, torque );
}

//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double ODEHingeJoint::GetParam( int parameter ) const
{
  double result = dJointGetHingeParam( this->jointId, parameter );

  return result; 
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter to _value
void ODEHingeJoint::SetParam( int parameter, double value)
{
  ODEJoint::SetParam(parameter, value);

  dJointSetHingeParam( this->jointId, parameter, value );
}
