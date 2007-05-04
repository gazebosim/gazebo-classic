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
/* Desc: A universal joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#include "World.hh"
#include "UniversalJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
UniversalJoint::UniversalJoint( dWorldID worldId )
  : Joint()
{
    this->jointId = dJointCreateUniversal( worldId, NULL );
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
UniversalJoint::~UniversalJoint()
{
}


//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
Vector3 UniversalJoint::GetAnchor() const
{
  dVector3 result;
  dJointGetUniversalAnchor( this->jointId, result );
  return Vector3( result[0], result[1], result[2] );
}


//////////////////////////////////////////////////////////////////////////////
// Get the first axis of rotation
Vector3 UniversalJoint::GetAxis1(  ) const
{
  dVector3 result;
  dJointGetUniversalAxis1( this->jointId, result );
  return Vector3( result[0], result[1], result[2] );
}


//////////////////////////////////////////////////////////////////////////////
// Get the second axis of rotation
Vector3 UniversalJoint::GetAxis2() const
{
  dVector3 result;
  dJointGetUniversalAxis2( this->jointId, result );
  return Vector3( result[0], result[1], result[2] );
}


//////////////////////////////////////////////////////////////////////////////
// Get the angle of axis 1
double UniversalJoint::GetAngle1() const
{
  return dJointGetUniversalAngle1( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the angle of axis 2
double UniversalJoint::GetAngle2() const
{
  return dJointGetUniversalAngle2( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the angular rate of axis 1
double UniversalJoint::GetAngleRate1() const
{
  return dJointGetUniversalAngle1Rate( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the angular rate of axis 2
double UniversalJoint::GetAngleRate2() const
{
  return dJointGetUniversalAngle2Rate( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void UniversalJoint::SetAnchor( const Vector3 &anchor )
{
  dJointSetUniversalAnchor( this->jointId, anchor.x, anchor.y, anchor.z );
}


//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void UniversalJoint::SetAxis1( const Vector3 &axis )
{
  dJointSetUniversalAxis1( this->jointId, axis.x, axis.y, axis.z );
}


//////////////////////////////////////////////////////////////////////////////
// Set the second axis of rotation
void UniversalJoint::SetAxis2( const Vector3 &axis )
{
  dJointSetUniversalAxis2( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the parameter to value
void UniversalJoint::SetParam( int parameter, double value )
{
  dJointSetUniversalParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void UniversalJoint::SetTorque(double torque1, double torque2)
{
  dJointAddUniversalTorques( this->jointId, torque1, torque2 );
}
