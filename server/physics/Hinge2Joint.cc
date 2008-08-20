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
  this->type = HINGE2;
  this->jointId = dJointCreateHinge2( worldId, NULL );

  this->axis1P = new Param<Vector3>("axis1",Vector3(0,0,1), 0);
  this->axis2P = new Param<Vector3>("axis2",Vector3(0,0,1), 0);
  this->loStop1P = new Param<Angle>("lowStop1",-M_PI,0);
  this->hiStop1P = new Param<Angle>("highStop1",M_PI,0);
  this->loStop2P = new Param<Angle>("lowStop2",-M_PI,0);
  this->hiStop2P = new Param<Angle>("highStop2",M_PI,0);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
Hinge2Joint::~Hinge2Joint()
{
  delete this->axis1P;
  delete this->axis2P;
  delete this->loStop1P;
  delete this->hiStop1P;
  delete this->loStop2P;
  delete this->hiStop2P;
}

//////////////////////////////////////////////////////////////////////////////
///  Load the joint
void Hinge2Joint::LoadChild(XMLConfigNode *node)
{
  this->axis1P->Load(node);
  this->axis2P->Load(node);
  this->loStop1P->Load(node);
  this->hiStop1P->Load(node);
  this->loStop2P->Load(node);
  this->hiStop2P->Load(node);

  this->SetAxis1(**(this->axis1P));
  this->SetAxis2(**(this->axis2P));

  // Perform this three step ordering to ensure the parameters are set
  // properly. This is taken from the ODE wiki.
  this->SetParam(dParamHiStop, this->hiStop1P->GetValue().GetAsRadian());
  this->SetParam(dParamLoStop, this->loStop1P->GetValue().GetAsRadian());
  this->SetParam(dParamHiStop, this->hiStop1P->GetValue().GetAsRadian());

  // Perform this three step ordering to ensure the parameters are set
  // properly. This is taken from the ODE wiki.
  this->SetParam(dParamHiStop2, this->hiStop2P->GetValue().GetAsRadian());
  this->SetParam(dParamLoStop2, this->loStop2P->GetValue().GetAsRadian());
  this->SetParam(dParamHiStop2, this->hiStop2P->GetValue().GetAsRadian());
}

//////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void Hinge2Joint::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->axis1P) << "\n";
  stream << prefix << *(this->loStop1P) << "\n";
  stream << prefix << *(this->hiStop1P) << "\n";

  stream << prefix << *(this->axis2P) << "\n";
  stream << prefix << *(this->loStop2P) << "\n";
  stream << prefix << *(this->hiStop2P) << "\n";
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
