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
  this->type = Joint::SLIDER;
  this->jointId = dJointCreateSlider( worldId, NULL );

  Param::Begin(&this->parameters);
  this->axisP = new ParamT<Vector3>("axis",Vector3(0,0,1), 0);
  this->loStopP = new ParamT<double>("lowStop",-DBL_MAX,0);
  this->hiStopP = new ParamT<double>("highStop",DBL_MAX,0);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SliderJoint::~SliderJoint()
{
  delete this->axisP;
  delete this->loStopP;
  delete this->hiStopP;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void SliderJoint::LoadChild(XMLConfigNode *node)
{
  this->axisP->Load(node);
  this->loStopP->Load(node);
  this->hiStopP->Load(node);

  this->SetAxis(**(this->axisP));

  // Perform this three step ordering to ensure the parameters are set
  // properly. This is taken from the ODE wiki.
  this->SetParam(dParamHiStop, **(this->hiStopP));
  this->SetParam(dParamLoStop, **(this->loStopP));
  this->SetParam(dParamHiStop, **(this->hiStopP));
}

//////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void SliderJoint::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->axisP) << "\n";
  stream << prefix << *(this->loStopP) << "\n";
  stream << prefix << *(this->hiStopP) << "\n";
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
void SliderJoint::SetAxis( const Vector3 &axis )
{
  dJointSetSliderAxis( this->jointId, axis.x, axis.y, axis.z );
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
