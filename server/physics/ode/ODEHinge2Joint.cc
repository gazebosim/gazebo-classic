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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODEHinge2Joint.cc 7129 2008-11-12 19:38:15Z natepak $
 */

#include "Body.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "ODEHinge2Joint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEHinge2Joint::ODEHinge2Joint( dWorldID worldId )
    : Hinge2Joint<ODEJoint>()
{
  this->jointId = dJointCreateHinge2( worldId, NULL );

  Param::Begin(&this->parameters);
  this->suspensionCfmP = new ParamT<double>("suspensionCfm",0.0,0);
  Param::End();
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEHinge2Joint::~ODEHinge2Joint()
{
  delete this->suspensionCfmP;
}

//////////////////////////////////////////////////////////////////////////////
///  Load the joint
void ODEHinge2Joint::Load(XMLConfigNode *node)
{
  Hinge2Joint<ODEJoint>::Load(node);

  this->suspensionCfmP->Load(node);

  // Suspension CFM is only valid for Hinge2 joints
  this->SetParam(dParamSuspensionCFM, **(this->suspensionCfmP));
}

//////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void ODEHinge2Joint::SaveJoint(std::string &prefix, std::ostream &stream)
{
  Hinge2Joint<ODEJoint>::SaveJoint(prefix, stream);
  stream << prefix << "  " << *(this->suspensionCfmP) << "\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get anchor point
Vector3 ODEHinge2Joint::GetAnchor(int index) const
{
  dVector3 result;

  this->physics->LockMutex();
  if (index == 0)
    dJointGetHinge2Anchor( this->jointId, result );
  else
    dJointGetHinge2Anchor2( this->jointId, result );
  this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void ODEHinge2Joint::SetAnchor( int /*index*/, const Vector3 &anchor )
{
  this->physics->LockMutex();
  this->body1->SetEnabled(true);
  this->body2->SetEnabled(true);
  dJointSetHinge2Anchor( this->jointId, anchor.x, anchor.y, anchor.z );
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void ODEHinge2Joint::SetAxis( int index, const Vector3 &axis )
{
  this->physics->LockMutex();
  this->body1->SetEnabled(true);
  this->body2->SetEnabled(true);

  if (index == 0)
    dJointSetHinge2Axis1( this->jointId, axis.x, axis.y, axis.z );
  else
    dJointSetHinge2Axis2( this->jointId, axis.x, axis.y, axis.z );
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Get first axis of rotation
Vector3 ODEHinge2Joint::GetAxis(int index) const
{
  dVector3 result;

  this->physics->LockMutex();
  if (index == 0)
    dJointGetHinge2Axis1( this->jointId, result );
  else
    dJointGetHinge2Axis2( this->jointId, result );
  this->physics->UnlockMutex();

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get angle of rotation about first axis
Angle ODEHinge2Joint::GetAngle(int index) const
{
  this->physics->LockMutex();
  if (index == 0)
    return dJointGetHinge2Angle1( this->jointId );
  else
    gzerr(0) << "ODE has not function to get the second angle in a hinge2 joint";
  this->physics->UnlockMutex();

  return Angle(0);
}

//////////////////////////////////////////////////////////////////////////////
// Get rate of rotation about first axis
double ODEHinge2Joint::GetVelocity(int index) const
{
  double result;

  this->physics->LockMutex();
  if (index == 0)
    result = dJointGetHinge2Angle1Rate( this->jointId );
  else
    result = dJointGetHinge2Angle2Rate( this->jointId );
  this->physics->UnlockMutex();
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
  this->physics->LockMutex();
  this->body1->SetEnabled(true);
  this->body2->SetEnabled(true);

  if (index == 0)
    dJointAddHinge2Torques(this->jointId, torque, 0);
  else
    dJointAddHinge2Torques(this->jointId, 0, torque);
  this->physics->UnlockMutex();
}

//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double ODEHinge2Joint::GetParam( int parameter ) const
{
  this->physics->LockMutex();
  double result = dJointGetHinge2Param( this->jointId, parameter );
  this->physics->UnlockMutex();

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Set _parameter with _value
void ODEHinge2Joint::SetParam( int parameter, double value)
{
  this->physics->LockMutex();
  ODEJoint::SetParam(parameter, value);
  dJointSetHinge2Param( this->jointId, parameter, value );
  this->physics->UnlockMutex();
}
