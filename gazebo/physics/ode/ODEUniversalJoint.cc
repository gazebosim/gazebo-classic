/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
math::Vector3 ODEUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetUniversalAnchor(this->jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  if (this->jointId)
    dJointSetUniversalAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Vector3 ODEUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  dVector3 result;

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == 1)
      dJointGetUniversalAxis1(this->jointId, result);
    else if (_index == 0)
      dJointGetUniversalAxis2(this->jointId, result);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
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
    // flipping axis 1 and 2 around
    if (_index == 1)
      dJointSetUniversalAxis1(this->jointId,
        globalAxis.x, globalAxis.y, globalAxis.z);
    else if (_index == 0)
      dJointSetUniversalAxis2(this->jointId,
        globalAxis.x, globalAxis.y, globalAxis.z);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Angle ODEUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == 1)
      result = dJointGetUniversalAngle1(this->jointId);
    else if (_index == 0)
      result = dJointGetUniversalAngle2(this->jointId);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == 1)
      result = dJointGetUniversalAngle1Rate(this->jointId);
    else if (_index == 0)
      result = dJointGetUniversalAngle2Rate(this->jointId);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetVelocity(unsigned int _index, double _angle)
{
  // flipping axis 1 and 2 around
  if (_index == 1)
    this->SetParam(dParamVel, _angle);
  else if (_index == 0)
    this->SetParam(dParamVel2, _angle);
  else
    gzerr << "Joint index out of bounds.\n";
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == 1)
      dJointAddUniversalTorques(this->jointId, _effort, 0);
    else if (_index == 0)
      dJointAddUniversalTorques(this->jointId, 0, _effort);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetMaxForce(unsigned int _index, double _t)
{
  // flipping axis 1 and 2 around
  if (_index == 1)
    this->SetParam(dParamFMax, _t);
  else if (_index == 0)
    this->SetParam(dParamFMax2, _t);
  else
    gzerr << "Joint index out of bounds.\n";
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetMaxForce(unsigned int _index)
{
  // flipping axis 1 and 2 around
  if (_index == 1)
    return this->GetParam(dParamFMax);
  else if (_index == 0)
    return this->GetParam(dParamFMax2);
  else
    gzerr << "Joint index out of bounds.\n";
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetParam(unsigned int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->jointId)
    dJointSetUniversalParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}
