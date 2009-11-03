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
/* Desc: A slider or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODESliderJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "XMLConfig.hh"
#include "ODESliderJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODESliderJoint::ODESliderJoint( dWorldID worldId )
    : SliderJoint<ODEJoint>()
{
  this->jointId = dJointCreateSlider( worldId, NULL );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODESliderJoint::~ODESliderJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void ODESliderJoint::Load(XMLConfigNode *node)
{
  SliderJoint<ODEJoint>::Load(node);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 ODESliderJoint::GetAxis(int /*index*/) const
{
  dVector3 result;
  dJointGetSliderAxis( this->jointId, result );

  return Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
Angle ODESliderJoint::GetAngle(int index) const
{
  return dJointGetSliderPosition( this->jointId );
}

//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double ODESliderJoint::GetVelocity(int /*index*/) const
{
  return dJointGetSliderPositionRate( this->jointId );
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODESliderJoint::SetVelocity(int index, double angle)
{
  this->SetParam(dParamVel, angle);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of motion
void ODESliderJoint::SetAxis( int /*index*/, const Vector3 &axis )
{
  dJointSetSliderAxis( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the slider force
void ODESliderJoint::SetForce(int /*index*/, double force)
{
  dJointAddSliderForce(this->jointId, force);
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter
void ODESliderJoint::SetParam( int parameter, double value )
{
  dJointSetSliderParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Get the _parameter
double ODESliderJoint::GetParam( int parameter ) const
{
  return dJointGetSliderParam( this->jointId, parameter );
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODESliderJoint::SetMaxForce(int index, double t) 
{
  this->SetParam(dParamFMax, t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODESliderJoint::GetMaxForce(int index)
{
  this->GetParam(dParamFMax);
}

