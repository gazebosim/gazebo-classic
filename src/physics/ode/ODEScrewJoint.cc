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
/* Desc: A screw or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: ODEScrewJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include <boost/bind.hpp>

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/ode/ODEScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEScrewJoint::ODEScrewJoint( dWorldID worldId )
    : ScrewJoint<ODEJoint>()
{
  this->jointId = dJointCreateScrew( worldId, NULL );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEScrewJoint::~ODEScrewJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void ODEScrewJoint::Load( sdf::ElementPtr &_sdf )
{
  ScrewJoint<ODEJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
math::Vector3 ODEScrewJoint::GetGlobalAxis(int /*index*/) const
{
  dVector3 result;
  dJointGetScrewAxis( this->jointId, result );

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
math::Angle ODEScrewJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result = dJointGetScrewPosition( this->jointId );
  
  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double ODEScrewJoint::GetVelocity(int /*index*/) const
{
  double result = dJointGetScrewPositionRate( this->jointId );

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void ODEScrewJoint::SetVelocity(int /*index*/, double angle)
{
  this->SetParam(dParamVel, angle);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of motion
void ODEScrewJoint::SetAxis( int /*index*/, const math::Vector3 &axis )
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  dJointSetScrewAxis( this->jointId, axis.x, axis.y, axis.z );
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void ODEScrewJoint::SetDamping( int /*index*/, const double damping )
{
  this->damping_coefficient = damping;
  dJointSetDamping( this->jointId, this->damping_coefficient);
}

//////////////////////////////////////////////////////////////////////////////
// Set thread pitch
void ODEScrewJoint::SetThreadPitch( int /*index*/, const double _thread_pitch )
{
  //std::cout << "\n\n\n\nsetting threadPitch: " << _thread_pitch << "\n\n\n\n\n";
  dJointSetScrewThreadPitch( this->jointId, _thread_pitch);
}

//////////////////////////////////////////////////////////////////////////////
// callback to apply joint damping force
void ODEScrewJoint::ApplyDamping()
{
  double damping_force = this->damping_coefficient * this->GetVelocity(0);
  this->SetForce(0,damping_force);
}

//////////////////////////////////////////////////////////////////////////////
// Set the screw force
void ODEScrewJoint::SetForce(int /*index*/, double force)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
  //dJointAddScrewForce(this->jointId, force);
  dJointAddScrewTorque(this->jointId, force);
}

//////////////////////////////////////////////////////////////////////////////
// Set the _parameter
void ODEScrewJoint::SetParam( int parameter, double value )
{
  ODEJoint::SetParam(parameter, value);
  dJointSetScrewParam( this->jointId, parameter, value );
}

//////////////////////////////////////////////////////////////////////////////
// Get the _parameter
double ODEScrewJoint::GetParam( int parameter ) const
{
  double result = dJointGetScrewParam( this->jointId, parameter );

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void ODEScrewJoint::SetMaxForce(int /*_index*/, double _t) 
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double ODEScrewJoint::GetMaxForce(int /*_index*/)
{
  return this->GetParam(dParamFMax);
}
