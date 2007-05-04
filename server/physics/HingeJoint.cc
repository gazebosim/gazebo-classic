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
/* Desc: A HingeJoint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#include "World.hh"
#include "HingeJoint.hh"

//////////////////////////////////////////////////////////////////////////////
// Constructor
HingeJoint::HingeJoint( dWorldID worldId )
  : Joint()
{
  this->jointId = dJointCreateHinge( worldId, NULL );
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
HingeJoint::~HingeJoint()
{
}


// Get the anchor point 
//////////////////////////////////////////////////////////////////////////////
Vector3 HingeJoint::GetAnchor( ) const
{
  dVector3 result;
  dJointGetHingeAnchor( this->jointId, result );

  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 HingeJoint::GetAxis() const
{
  dVector3 result;
  dJointGetHingeAxis( this->jointId, result );

  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get the angle of rotation
double HingeJoint::GetAngle() const
{
  return dJointGetHingeAngle( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the rotation rate
double HingeJoint::GetAngleRate() const
{
  return dJointGetHingeAngleRate( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double HingeJoint::GetParam( int parameter ) const
{
  return dJointGetHingeParam( this->jointId, parameter );
}


//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void HingeJoint::SetAnchor( const Vector3 &anchor )
{
  dJointSetHingeAnchor( this->jointId, anchor.x, anchor.y, anchor.z );
}


//////////////////////////////////////////////////////////////////////////////
// Set the axis of rotation
void HingeJoint::SetAxis( const Vector3 &axis )
{
 dJointSetHingeAxis( this->jointId, axis.x, axis.y, axis.z );
}



//////////////////////////////////////////////////////////////////////////////
// Set the _parameter to _value
void HingeJoint::SetParam( int parameter, double value )
{
  dJointSetHingeParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void HingeJoint::SetTorque(double torque)
{
  dJointAddHingeTorque( this->jointId, torque );
}
