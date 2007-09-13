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
 * CVS: $Id$
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Hinge2Joint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
Hinge2Joint::Hinge2Joint( dWorldID worldId )
  : Joint()
{
  this->jointId = dJointCreateHinge2( worldId, NULL );
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
Hinge2Joint::~Hinge2Joint()
{
}

//////////////////////////////////////////////////////////////////////////////
///  Load the joint
void Hinge2Joint::LoadChild(XMLConfigNode *node)
{
  this->SetAxis1(node->GetVector3("axis1",Vector3(0,0,1)));
  this->SetAxis2(node->GetVector3("axis2",Vector3(0,0,1)));

  double loStop1 = node->GetDouble("lowStop1",-DBL_MAX,0);
  double hiStop1 = node->GetDouble("highStop1",DBL_MAX,0);

  double loStop2 = node->GetDouble("lowStop2",-DBL_MAX,0);
  double hiStop2 = node->GetDouble("highStop2",DBL_MAX,0);

  if (loStop1 != -DBL_MAX)
  {
    loStop1 = DTOR(loStop1);
    this->SetParam(dParamLoStop, loStop1);
  }

  if (hiStop1 != DBL_MAX)
  {
    hiStop1 = DTOR(hiStop1);
    this->SetParam(dParamHiStop, hiStop1);
  }

  if (loStop2 != -DBL_MAX)
  {
    loStop2 = DTOR(loStop2);
    this->SetParam(dParamLoStop2, loStop1);
  }

  if (hiStop2 != DBL_MAX)
  {
    hiStop2 = DTOR(hiStop2);
    this->SetParam(dParamHiStop2, hiStop2);
  }

}
 

//////////////////////////////////////////////////////////////////////////////
// Get anchor point
Vector3 Hinge2Joint::GetAnchor() const
{
  dVector3 result;
  dJointGetHinge2Anchor( this->jointId, result );
  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get the second anchor point
Vector3 Hinge2Joint::GetAnchor2() const
{ 
  dVector3 result; 
  dJointGetHinge2Anchor2( this->jointId, result );
  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get first axis of rotation
Vector3 Hinge2Joint::GetAxis1() const
{
  dVector3 result; 
  dJointGetHinge2Axis1( this->jointId, result );
  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get second axis of rotation
Vector3 Hinge2Joint::GetAxis2() const
{
  dVector3 result; 
  dJointGetHinge2Axis2( this->jointId, result );
  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get angle of rotation about first axis
double Hinge2Joint::GetAngle1() const
{
  return dJointGetHinge2Angle1( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get rate of rotation about first axis
double Hinge2Joint::GetAngle1Rate() const
{
  return dJointGetHinge2Angle1Rate( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get rate of rotation about second axis
double Hinge2Joint::GetAngle2Rate() const
{
  return dJointGetHinge2Angle2Rate( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the specified parameter
double Hinge2Joint::GetParam( int parameter ) const
{
  return dJointGetHinge2Param( this->jointId, parameter );
}


//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void Hinge2Joint::SetAnchor( const Vector3 &anchor )
{
  dJointSetHinge2Anchor( this->jointId, anchor.x, anchor.y, anchor.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void Hinge2Joint::SetAxis1( const Vector3 &axis )
{
  dJointSetHinge2Axis1( this->jointId, axis.x, axis.y, axis.z );
}


//////////////////////////////////////////////////////////////////////////////
// Set the second axis of rotation
void Hinge2Joint::SetAxis2( const Vector3 &axis )
{
  dJointSetHinge2Axis2( this->jointId, axis.x, axis.y, axis.z );
}


//////////////////////////////////////////////////////////////////////////////
// Set _parameter with _value
void Hinge2Joint::SetParam( int parameter, double value)
{
  dJointSetHinge2Param( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Set _parameter with _value
void Hinge2Joint::SetTorque(double torque1, double torque2)
{
  dJointAddHinge2Torques(this->jointId, torque1, torque2);
}
