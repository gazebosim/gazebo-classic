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
#include "Global.hh"
#include "XMLConfig.hh"
#include "UniversalJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
UniversalJoint::UniversalJoint( dWorldID worldId )
    : Joint()
{
  this->type = Joint::UNIVERSAL;
  this->jointId = dJointCreateUniversal( worldId, NULL );

  Param::Begin(&this->parameters);
  this->axis1P = new ParamT<Vector3>("axis1",Vector3(0,0,1),0);
  this->axis2P = new ParamT<Vector3>("axis2",Vector3(0,0,1),0);

  this->loStop1P = new ParamT<Angle>("lowStop1",-M_PI,0);
  this->hiStop1P = new ParamT<Angle>("highStop1",M_PI,0);
  this->loStop2P = new ParamT<Angle>("lowStop2",-M_PI,0);
  this->hiStop2P = new ParamT<Angle>("highStop2",M_PI,0);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
UniversalJoint::~UniversalJoint()
{
  delete this->axis1P;
  delete this->axis2P;
  delete this->loStop1P;
  delete this->hiStop1P;
  delete this->loStop2P;
  delete this->hiStop2P;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void UniversalJoint::LoadChild(XMLConfigNode *node)
{
  this->axis1P->Load(node);
  this->axis2P->Load(node);

  this->loStop1P->Load(node);
  this->hiStop1P->Load(node);
  this->loStop2P->Load(node);
  this->hiStop2P->Load(node);

  this->SetAxis1(**(this->axis1P));
  this->SetAxis2(**(this->axis1P));

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
void UniversalJoint::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->axis1P) << "\n";
  stream << prefix << *(this->loStop1P) << "\n";
  stream << prefix << *(this->hiStop1P) << "\n";

  stream << prefix << *(this->axis2P) << "\n";
  stream << prefix << *(this->loStop2P) << "\n";
  stream << prefix << *(this->hiStop2P) << "\n";
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
void UniversalJoint::SetParam( int parameter, double value)
{
  dJointSetUniversalParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void UniversalJoint::SetTorque(double torque1, double torque2)
{
  dJointAddUniversalTorques( this->jointId, torque1, torque2 );
}
