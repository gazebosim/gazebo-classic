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
#include <string>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginUniversalJoint::PluginUniversalJoint(dWorldID _worldId, BasePtr _parent)
    : UniversalJoint<PluginJoint>(_parent)
{
  this->jointId = 0;
}

//////////////////////////////////////////////////
PluginUniversalJoint::~PluginUniversalJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
math::Vector3 PluginUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  dVector3 result;

  // get joint anchor

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  // set joint anchor
}

//////////////////////////////////////////////////
math::Vector3 PluginUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  dVector3 result;

  if (this->jointId)
  {
    // get joint axis
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// Plugin needs global axis
  math::Quaternion axisFrame = this->GetAxisFrame(_index);
  math::Vector3 globalAxis = axisFrame.RotateVector(_axis);

  if (this->jointId)
  {
    // set joint axis
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Angle PluginUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (this->jointId)
  {
    // get joint angle
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double PluginUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    // get joint velocity
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetVelocity(unsigned int _index, double _angle)
{
  // flipping axis 1 and 2 around
  if (_index == UniversalJoint::AXIS_CHILD)
    this->SetParam(dParamVel, _angle);
  else if (_index == UniversalJoint::AXIS_PARENT)
    this->SetParam(dParamVel2, _angle);
  else
    gzerr << "Joint index out of bounds.\n";
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->jointId)
  {
    // set joint force
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetMaxForce(unsigned int _index, double _t)
{
  // set joint max force
}

//////////////////////////////////////////////////
double PluginUniversalJoint::GetMaxForce(unsigned int _index)
{
  // get joint max force
  return 0;
}

//////////////////////////////////////////////////
bool PluginUniversalJoint::SetHighStop(
  unsigned int _index, const math::Angle &_angle)
{
  // Overload because we switched axis orders
  Joint::SetHighStop(_index, _angle);
  // set joint high stop
}

//////////////////////////////////////////////////
bool PluginUniversalJoint::SetLowStop(
  unsigned int _index, const math::Angle &_angle)
{
  // Overload because we switched axis orders
  Joint::SetLowStop(_index, _angle);
  // set joint low stop
}

//////////////////////////////////////////////////
bool PluginUniversalJoint::SetParam(
  const std::string &_key, unsigned int _index, const boost::any &_value)
{
  if (_key == "stop_erp")
  {
    // set joint stop erp
  }
  else if (_key == "stop_cfm")
  {
    // set joint stop erp
  }
  else if (_key == "hi_stop")
  {
    // set high stop
  }
  else if (_key == "lo_stop")
  {
    // set low stop
  }
  else
  {
    // Overload because we switched axis orders
    return PluginJoint::SetParam(_key, _index, _value);
  }
  return true;
}

//////////////////////////////////////////////////
double PluginUniversalJoint::GetParam(
  const std::string &_key, unsigned int _index)
{
  // Overload because we switched axis orders
  if (_key == "hi_stop")
  {
    // return joint high stop
    return 0;
  }
  else if (_key == "lo_stop")
  {
    // return joint low stop
    return 0;
  }
  else
  {
    return PluginJoint::GetParam(_key, _index);
  }
}
