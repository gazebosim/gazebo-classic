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
 * CVS: $Id: ODEHingeJoint.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include "gazebo_config.h"
#include "GazeboMessage.hh"
#include "Model.hh"
#include "Body.hh"
#include "World.hh"
#include "XMLConfig.hh"
#include "Global.hh"
#include "ODEHingeJoint.hh"
#include <boost/signal.hpp>
#include <boost/bind.hpp>


using namespace gazebo;

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
void ODEHingeJoint::Load(XMLConfigNode *node)
{
  HingeJoint<ODEJoint>::Load(node);

  this->SetParam(dParamFMax, 0);
  this->SetForce(0, 0);
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
Vector3 ODEHingeJoint::GetAnchor(int /*index*/) const
{
  dVector3 result;

  // NATY
  //this->physics->LockMutex();
  dJointGetHingeAnchor( this->jointId, result );
  // NATY
  //this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void ODEHingeJoint::SetAnchor( int /*index*/, const Vector3 &anchor )
{
  // NATY
  //this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

  dJointSetHingeAnchor( this->jointId, anchor.x, anchor.y, anchor.z );
  // NATY
  //this->physics->UnlockMutex();
}


//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 ODEHingeJoint::GetAxis(int /*index*/) const
{
  dVector3 result;

  // NATY
  //this->physics->LockMutex();
  dJointGetHingeAxis( this->jointId, result );
  // NATY
  //this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of rotation
void ODEHingeJoint::SetAxis( int /*index*/, const Vector3 &axis )
{
  // NATY
  //this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);

  dJointSetHingeAxis( this->jointId, axis.x, axis.y, axis.z );
  // NATY
  //this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping, either through ODE or callback here
void ODEHingeJoint::SetDamping( int /*index*/, const double damping )
{
  this->damping_coefficient = damping;
#ifdef INCLUDE_ODE_JOINT_DAMPING
  // NATY
  //this->physics->LockMutex();
  dJointSetDamping( this->jointId, this->damping_coefficient);
  // NATY
  //this->physics->UnlockMutex();
#else
  // alternaitvely, apply explicit damping
  this->ConnectJointUpdateSignal(boost::bind(&ODEHingeJoint::ApplyDamping,this));
#endif
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
Angle ODEHingeJoint::GetAngle(int /*index*/) const
{
  // NATY
  //this->physics->LockMutex();
  Angle result = dJointGetHingeAngle( this->jointId );
  // NATY
  //this->physics->UnlockMutex();

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the rotation rate
double ODEHingeJoint::GetVelocity(int /*index*/) const
{
  // NATY
  //this->physics->LockMutex();
  double result = dJointGetHingeAngleRate( this->jointId );
  // NATY
  //this->physics->UnlockMutex();

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
  // NATY
  //this->physics->LockMutex();
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);
  dJointAddHingeTorque( this->jointId, torque );
  // NATY
  //this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double ODEHingeJoint::GetParam( int parameter ) const
{
  // NATY
  //this->physics->LockMutex();
  double result = dJointGetHingeParam( this->jointId, parameter );
  // NATY
  //this->physics->UnlockMutex();

  return result; 
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter to _value
void ODEHingeJoint::SetParam( int parameter, double value)
{
  // NATY
  //this->physics->LockMutex();
  ODEJoint::SetParam(parameter, value);

  dJointSetHingeParam( this->jointId, parameter, value );
  // NATY
  //this->physics->UnlockMutex();
}
