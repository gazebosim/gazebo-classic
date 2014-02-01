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
/* Desc: A universal joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEUniversalJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODEUniversalJoint::ODEUniversalJoint(dWorldID _worldId, BasePtr _parent)
    : UniversalJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateUniversal(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEUniversalJoint::~ODEUniversalJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
math::Vector3 ODEUniversalJoint::GetAnchor(int /*index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetUniversalAnchor(this->jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAnchor(int /*index*/, const math::Vector3 &_anchor)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  if (this->jointId)
    dJointSetUniversalAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Vector3 ODEUniversalJoint::GetGlobalAxis(int _index) const
{
  dVector3 result;

  if (this->jointId)
  {
    if (_index == 0)
      dJointGetUniversalAxis1(this->jointId, result);
    else
      dJointGetUniversalAxis2(this->jointId, result);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAxis(int _index, const math::Vector3 &_axis)
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
  {
    if (_index == 0)
      dJointSetUniversalAxis1(this->jointId,
        globalAxis.x, globalAxis.y, globalAxis.z);
    else
      dJointSetUniversalAxis2(this->jointId,
        globalAxis.x, globalAxis.y, globalAxis.z);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Angle ODEUniversalJoint::GetAngleImpl(int _index) const
{
  math::Angle result;

  if (this->jointId)
  {
    if (_index == 0)
      result = dJointGetUniversalAngle1(this->jointId);
    else
      result = dJointGetUniversalAngle2(this->jointId);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetVelocity(int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    if (_index == 0)
      result = dJointGetUniversalAngle1Rate(this->jointId);
    else
      result = dJointGetUniversalAngle2Rate(this->jointId);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetVelocity(int _index, double _angle)
{
  if (_index == 0)
    this->SetParam(dParamVel, _angle);
  else
    this->SetParam(dParamVel2, _angle);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetForceImpl(int _index, double _effort)
{
  if (this->jointId)
  {
    if (_index == 0)
      dJointAddUniversalTorques(this->jointId, _effort, 0);
    else
      dJointAddUniversalTorques(this->jointId, 0, _effort);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetMaxForce(int _index, double _t)
{
  if (_index == 0)
    this->SetParam(dParamFMax, _t);
  else
    this->SetParam(dParamFMax2, _t);
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetMaxForce(int _index)
{
  if (_index == 0)
    return this->GetParam(dParamFMax);
  else
    return this->GetParam(dParamFMax2);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetParam(int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->jointId)
    dJointSetUniversalParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}
