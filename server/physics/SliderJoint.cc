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
 * CVS: $Id$
 */

#include "XMLConfig.hh"
#include "SliderJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
SliderJoint::SliderJoint( dWorldID worldId )
  : Joint()
{
  this->jointId = dJointCreateSlider( worldId, NULL );
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SliderJoint::~SliderJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void SliderJoint::LoadChild(XMLConfigNode *node)
{
  double lowStop = node->GetDouble("lowStop",-DBL_MAX,0);
  double hiStop = node->GetDouble("hiStop",DBL_MAX,0);

  // Perform this three step ordering to ensure the parameters are set
  // properly. This is taken from the ODE wiki.
  this->SetParam(dParamHiStop, hiStop);
  this->SetParam(dParamLoStop, lowStop);
  this->SetParam(dParamHiStop, hiStop);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 SliderJoint::GetAxis() const
{
  dVector3 result;
  dJointGetSliderAxis( this->jointId, result );

  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
double SliderJoint::GetPosition() const
{
  return dJointGetSliderPosition( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double SliderJoint::GetPositionRate() const
{
  return dJointGetSliderPositionRate( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the _parameter
double SliderJoint::GetParam( int parameter ) const
{
  return dJointGetSliderParam( this->jointId, parameter );
}


//////////////////////////////////////////////////////////////////////////////
// Set the axis of motion
void SliderJoint::SetAxis( double x, double y, double z)
{
  dJointSetSliderAxis( this->jointId, x, y, z );
}


//////////////////////////////////////////////////////////////////////////////
// Set the _parameter
void SliderJoint::SetParam( int parameter, double value )
{
  dJointSetSliderParam( this->jointId, parameter, value );
}
//////////////////////////////////////////////////////////////////////////////
// Set the slider force
void SliderJoint::SetSliderForce(double force)
{
  dJointAddSliderForce(this->jointId, force);
}
