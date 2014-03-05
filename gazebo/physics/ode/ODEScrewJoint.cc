/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/physics/Model.hh"
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

  if (this->jointId)
    dJointGetScrewAxis(this->jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
math::Angle ODEScrewJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->jointId)
    result = dJointGetScrewPosition(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetVelocity(int /*index*/) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetScrewPositionRate(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

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

  /// ODE needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  math::Vector3 globalAxis = _axis;
  if (this->parentLink)
    globalAxis =
      this->GetParent()->GetModel()->GetWorldPose().rot.RotateVector(_axis);

  if (this->jointId)
    dJointSetScrewAxis(this->jointId, globalAxis.x, globalAxis.y, globalAxis.z);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(int /*_index*/, double _threadPitch)
{
  if (this->jointId)
    dJointSetScrewThreadPitch(this->jointId, _threadPitch);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  gzerr << "not yet implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetForceImpl(int /*_index*/, double _effort)
{
  if (this->jointId)
  {
    // dJointAddScrewForce(this->jointId, _effort);
    dJointAddScrewTorque(this->jointId, _effort);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetParam(int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  if (this->jointId)
    dJointSetScrewParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetParam(int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetScrewParam(this->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

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
