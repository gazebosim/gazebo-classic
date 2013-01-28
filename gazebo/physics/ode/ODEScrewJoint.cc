/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Desc: A screw or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEScrewJoint.hh"

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
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
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
void ODEScrewJoint::SetDamping(int /*index*/, double _damping)
{
  this->dampingCoefficient = _damping;
  // dJointSetDamping(this->jointId, this->dampingCoefficient);
  this->applyDamping = physics::Joint::ConnectJointUpdate(
    boost::bind(&Joint::ApplyDamping, this));
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(int /*_index*/, double _threadPitch)
{
  dJointSetScrewThreadPitch(this->jointId, _threadPitch);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetForce(int _index, double _force)
{
  ODEJoint::SetForce(_index, _force);
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





