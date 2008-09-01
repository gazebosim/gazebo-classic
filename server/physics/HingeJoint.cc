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
#include "XMLConfig.hh"
#include "Global.hh"
#include "HingeJoint.hh"


using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
HingeJoint::HingeJoint( dWorldID worldId )
    : Joint()
{
  this->type = Joint::HINGE;
  this->jointId = dJointCreateHinge( worldId, NULL );

  Param::Begin(&this->parameters);
  this->axisP = new ParamT<Vector3>("axis",Vector3(0,1,0), 1);
  this->loStopP = new ParamT<Angle>("lowStop",-M_PI,0);
  this->hiStopP = new ParamT<Angle>("highStop",M_PI,0);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
HingeJoint::~HingeJoint()
{
  delete this->axisP;
  delete this->loStopP;
  delete this->hiStopP;
}

//////////////////////////////////////////////////////////////////////////////
/// Load a hinge joint
void HingeJoint::LoadChild(XMLConfigNode *node)
{
  this->axisP->Load(node);
  this->loStopP->Load(node);
  this->hiStopP->Load(node);

  // Perform this three step ordering to ensure the parameters are set
  // properly. This is taken from the ODE wiki.
  this->SetParam(dParamHiStop, this->hiStopP->GetValue().GetAsRadian());
  this->SetParam(dParamLoStop, this->loStopP->GetValue().GetAsRadian());
  this->SetParam(dParamHiStop, this->hiStopP->GetValue().GetAsRadian());

  this->SetAxis(**(this->axisP));
}

////////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void HingeJoint::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->axisP) << "\n";
  stream << prefix << *(this->loStopP) << "\n";
  stream << prefix << *(this->hiStopP) << "\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
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
void HingeJoint::SetParam( int parameter, double value)
{
  dJointSetHingeParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void HingeJoint::SetTorque(double torque)
{
  dJointAddHingeTorque( this->jointId, torque );
}
