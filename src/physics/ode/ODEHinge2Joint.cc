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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODEHinge2Joint.cc 7129 2008-11-12 19:38:15Z natepak $
 */

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/ode/ODEHinge2Joint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEHinge2Joint::ODEHinge2Joint( dWorldID worldId )
    : Hinge2Joint<ODEJoint>()
{
  this->jointId = dJointCreateHinge2( worldId, NULL );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEHinge2Joint::~ODEHinge2Joint()
{
}

//////////////////////////////////////////////////////////////////////////////
///  Load the joint
void ODEHinge2Joint::Load( sdf::ElementPtr &_sdf )
{
  Hinge2Joint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
// Get anchor point
math::Vector3 ODEHinge2Joint::GetAnchor(int index) const
{
  dVector3 result;

  if (index == 0)
    dJointGetHinge2Anchor( this->jointId, result );
  else
    dJointGetHinge2Anchor2( this->jointId, result );

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void ODEHinge2Joint::SetAnchor( int /*index*/, const math::Vector3 &anchor )
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
  dJointSetHinge2Anchor( this->jointId, anchor.x, anchor.y, anchor.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void ODEHinge2Joint::SetAxis( int index, const math::Vector3 &axis )
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  if (index == 0)
    dJointSetHinge2Axis1( this->jointId, axis.x, axis.y, axis.z );
  else
    dJointSetHinge2Axis2( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODEHinge2Joint::SetDamping( int /*_index*/, const double _damping )
{
  dJointSetDamping( this->jointId, _damping);
}

//////////////////////////////////////////////////////////////////////////////
// Get first axis of rotation
math::Vector3 ODEHinge2Joint::GetGlobalAxis(int index) const
{
  dVector3 result;

  if (index == 0)
    dJointGetHinge2Axis1( this->jointId, result );
  else
    dJointGetHinge2Axis2( this->jointId, result );

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get angle of rotation about first axis
math::Angle ODEHinge2Joint::GetAngleImpl(int index) const
{
  if (index == 0)
    return dJointGetHinge2Angle1( this->jointId );
  else
    gzerr << "ODE has not function to get the second angle in a hinge2 joint";

  return math::Angle(0);
}

//////////////////////////////////////////////////////////////////////////////
// Get rate of rotation about first axis
double ODEHinge2Joint::GetVelocity(int index) const
{
  double result;

  if (index == 0)
    result = dJointGetHinge2Angle1Rate( this->jointId );
  else
    result = dJointGetHinge2Angle2Rate( this->jointId );

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODEHinge2Joint::SetVelocity(int index, double angle)
{
  if (index == 0)
    this->SetParam(dParamVel, angle );
  else
    this->SetParam(dParamVel2, angle );
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODEHinge2Joint::GetMaxForce(int index)
{
  if (index == 0)
    return this->GetParam(dParamFMax);
  else
    return this->GetParam(dParamFMax2);
}


//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODEHinge2Joint::SetMaxForce(int index, double t)
{
  if (index == 0)
    this->SetParam(dParamFMax, t );
  else
    this->SetParam(dParamFMax2, t );
}


//////////////////////////////////////////////////////////////////////////////
// Set _parameter with _value
void ODEHinge2Joint::SetForce(int index, double torque)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  if (index == 0)
    dJointAddHinge2Torques(this->jointId, torque, 0);
  else
    dJointAddHinge2Torques(this->jointId, 0, torque);
}

//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double ODEHinge2Joint::GetParam( int parameter ) const
{
  double result = dJointGetHinge2Param( this->jointId, parameter );

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Set _parameter with _value
void ODEHinge2Joint::SetParam( int parameter, double value)
{
  ODEJoint::SetParam(parameter, value);
  dJointSetHinge2Param( this->jointId, parameter, value );
}
