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
/* Desc: A ODEHingeJoint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODEHingeJoint.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include "Model.hh"
#include "Body.hh"
#include "World.hh"
#include "XMLConfig.hh"
#include "Global.hh"
#include "ODEHingeJoint.hh"


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
  dJointGetHingeAnchor( this->jointId, result );

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void ODEHingeJoint::SetAnchor( int /*index*/, const Vector3 &anchor )
{
  dJointSetHingeAnchor( this->jointId, anchor.x, anchor.y, anchor.z );
}


//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 ODEHingeJoint::GetAxis(int /*index*/) const
{
  dVector3 result;
  dJointGetHingeAxis( this->jointId, result );

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of rotation
void ODEHingeJoint::SetAxis( int /*index*/, const Vector3 &axis )
{
  dJointSetHingeAxis( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Get the angle of rotation
Angle ODEHingeJoint::GetAngle(int /*index*/) const
{
  return dJointGetHingeAngle( this->jointId );
}

//////////////////////////////////////////////////////////////////////////////
// Get the rotation rate
double ODEHingeJoint::GetVelocity(int /*index*/) const
{
  return dJointGetHingeAngleRate( this->jointId );
} 

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODEHingeJoint::SetVelocity(int /*index*/, double angle)
{
  this->SetParam(angle, dParamVel);
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
  dJointAddHingeTorque( this->jointId, torque );
}

//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double ODEHingeJoint::GetParam( int parameter ) const
{
  return dJointGetHingeParam( this->jointId, parameter );
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter to _value
void ODEHingeJoint::SetParam( int parameter, double value)
{
  dJointSetHingeParam( this->jointId, parameter, value );
}
