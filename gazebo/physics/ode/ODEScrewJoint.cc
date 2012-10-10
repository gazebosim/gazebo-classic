/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig
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
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <boost/bind.hpp>

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/ode/ODEScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEScrewJoint::ODEScrewJoint(dWorldID _worldId, BasePtr _parent)
    : ScrewJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateScrew(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEScrewJoint::~ODEScrewJoint()
{
}

//////////////////////////////////////////////////
void ODEScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<ODEJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
math::Vector3 ODEScrewJoint::GetGlobalAxis(int /*index*/) const
{
  dVector3 result;
  dJointGetScrewAxis(this->jointId, result);

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
math::Angle ODEScrewJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->jointId)
    result = dJointGetScrewPosition(this->jointId);

  return result;
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetVelocity(int /*index*/) const
{
  double result = dJointGetScrewPositionRate(this->jointId);

  return result;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetVelocity(int /*index*/, double _angle)
{
  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  dJointSetScrewAxis(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetDamping(int /*index*/, const double _damping)
{
  this->damping_coefficient = _damping;
  dJointSetDamping(this->jointId, this->damping_coefficient);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(int /*index*/, const double _thread_pitch)
{
  dJointSetScrewThreadPitch(this->jointId, _thread_pitch);
}

//////////////////////////////////////////////////
void ODEScrewJoint::ApplyDamping()
{
  double damping_force = this->damping_coefficient * this->GetVelocity(0);
  this->SetForce(0, damping_force);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetForce(int /*index*/, double _force)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
  // dJointAddScrewForce(this->jointId, _force);
  dJointAddScrewTorque(this->jointId, _force);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetParam(int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  dJointSetScrewParam(this->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetParam(int _parameter) const
{
  double result = dJointGetScrewParam(this->jointId, _parameter);

  return result;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetMaxForce(int /*_index*/, double _t)
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetMaxForce(int /*_index*/)
{
  return this->GetParam(dParamFMax);
}





