/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODEHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEHinge2Joint::ODEHinge2Joint(dWorldID _worldId, BasePtr _parent)
: Hinge2Joint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateHinge2(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEHinge2Joint::~ODEHinge2Joint()
{
  if (this->jointDPtr->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->jointDPtr->applyDamping);
}

//////////////////////////////////////////////////
void ODEHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEHinge2Joint::Anchor(const unsigned int _index) const
{
  dVector3 result;

  if (this->odeJointDPtr->jointId)
  {
    if (_index == 0)
      dJointGetHinge2Anchor(this->odeJointDPtr->jointId, result);
    else
      dJointGetHinge2Anchor2(this->odeJointDPtr->jointId, result);
  }
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return math::Vector3::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);

  if (this->odeJointDPtr->jointId)
  {
    dJointSetHinge2Anchor(this->odeJointDPtr->jointId,
        _anchor.X(), _anchor.Y(), _anchor.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);

  /// ODE needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  ignition::math::Vector3d globalAxis = _axis;
  if (this->jointDPtr->parentLink)
  {
    globalAxis =
      this->Parent()->Model()->WorldPose().Rot().RotateVector(_axis);
  }

  if (this->odeJointDPtr->jointId)
  {
    if (_index == 0)
    {
      dJointSetHinge2Axis1(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
    }
    else
    {
      dJointSetHinge2Axis2(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
    }
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEHinge2Joint::GlobalAxis(
    const unsigned int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetHinge2Axis1(this->odeJointDPtr->jointId, result);
  else
    dJointGetHinge2Axis2(this->odeJointDPtr->jointId, result);

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
ignition::math::Angle ODEHinge2Joint::AngleImpl(
    const unsigned int _index) const
{
  ignition::math::Angle result;

  if (this->odeJointDPtr->jointId)
  {
    if (_index == 0)
      result = dJointGetHinge2Angle1(this->odeJointDPtr->jointId);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEHinge2Joint::Velocity(const unsigned int _index) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
  {
    if (_index == 0)
      result = dJointGetHinge2Angle1Rate(this->odeJointDPtr->jointId);
    else
      result = dJointGetHinge2Angle2Rate(this->odeJointDPtr->jointId);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetVelocity(const unsigned int _index, const double _angle)
{
  if (_index == 0)
    this->SetParam(dParamVel, _angle);
  else
    this->SetParam(dParamVel2, _angle);
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetForceImpl(const unsigned int _index,
    const double _effort)
{
  if (this->odeJointDPtr->jointId)
  {
    if (_index == 0)
      dJointAddHinge2Torques(this->odeJointDPtr->jointId, _effort, 0);
    else
      dJointAddHinge2Torques(this->odeJointDPtr->jointId, 0, _effort);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEHinge2Joint::Param(const unsigned int _parameter) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
    result = dJointGetHinge2Param(this->odeJointDPtr->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetParam(const unsigned int _parameter,
    const double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  if (this->odeJointDPtr->jointId)
    dJointSetHinge2Param(this->odeJointDPtr->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}
