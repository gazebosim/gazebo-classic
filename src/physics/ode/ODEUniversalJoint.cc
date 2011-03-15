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
/* Desc: A universal joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODEUniversalJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "gazebo_config.h"
#include "common/GazeboMessage.hh"

#include "physics/Body.hh"
#include "physics/ode/ODEUniversalJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEUniversalJoint::ODEUniversalJoint( dWorldID worldId )
    : UniversalJoint<ODEJoint>()
{
  this->jointId = dJointCreateUniversal( worldId, NULL );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEUniversalJoint::~ODEUniversalJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
common::Vector3 ODEUniversalJoint::GetAnchor(int /*index*/) const
{
  dVector3 result;
  dJointGetUniversalAnchor( this->jointId, result );

  return common::Vector3( result[0], result[1], result[2] );
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void ODEUniversalJoint::SetAnchor( int /*index*/, const common::Vector3 &anchor )
{
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

  dJointSetUniversalAnchor( this->jointId, anchor.x, anchor.y, anchor.z );
}

//////////////////////////////////////////////////////////////////////////////
// Get the first axis of rotation
common::Vector3 ODEUniversalJoint::GetAxis(int index ) const
{
  dVector3 result;

  if (index == 0)
    dJointGetUniversalAxis1( this->jointId, result );
  else
    dJointGetUniversalAxis2( this->jointId, result );

  return common::Vector3( result[0], result[1], result[2] );
}

//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void ODEUniversalJoint::SetAxis( int index, const common::Vector3 &axis )
{

  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

  if (index == 0)
    dJointSetUniversalAxis1( this->jointId, axis.x, axis.y, axis.z );
  else
    dJointSetUniversalAxis2( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODEUniversalJoint::SetDamping( int /*index*/, const double damping )
{
#ifdef INCLUDE_ODE_JOINT_DAMPING
  // ode does not yet support Universal joint damping
  dJointSetDamping( this->jointId, damping);
#else
  // alternaitvely, apply explicit damping
  gzerr(0) << "joint damping not implemented in ODE ball joint\n";
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Get the angle of an axis
common::Angle ODEUniversalJoint::GetAngle(int index) const
{
  common::Angle result;

  if (index == 0)
    result = dJointGetUniversalAngle1( this->jointId );
  else
    result = dJointGetUniversalAngle2( this->jointId );

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the angular rate of an axis
double ODEUniversalJoint::GetVelocity(int index) const
{
  double result;

  if (index == 0)
    result = dJointGetUniversalAngle1Rate( this->jointId );
  else 
    result = dJointGetUniversalAngle2Rate( this->jointId );
}
 
//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODEUniversalJoint::SetVelocity(int index,double angle)
{
  if (index == 0)
    this->SetParam(dParamVel, angle);
  else
    this->SetParam(dParamVel2, angle);
}
 
//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void ODEUniversalJoint::SetForce(int index, double torque)
{
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);
  if (index == 0)
    dJointAddUniversalTorques( this->jointId, torque, 0);
  else
    dJointAddUniversalTorques( this->jointId, 0, torque);
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODEUniversalJoint::SetMaxForce(int index, double t)
{
  if (index == 0)
    this->SetParam(dParamFMax, t);
  else
    this->SetParam(dParamFMax2, t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODEUniversalJoint::GetMaxForce(int index)
{
  if (index == 0)
    return this->GetParam(dParamFMax);
  else
    return this->GetParam(dParamFMax2);
}

//////////////////////////////////////////////////////////////////////////////
// Set the parameter to value
void ODEUniversalJoint::SetParam( int parameter, double value)
{
  ODEJoint::SetParam(parameter, value);
  dJointSetUniversalParam( this->jointId, parameter, value );
}


