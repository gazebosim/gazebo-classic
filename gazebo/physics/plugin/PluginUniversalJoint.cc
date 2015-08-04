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
PluginUniversalJoint::PluginUniversalJoint(BasePtr _parent)
    : UniversalJoint<PluginJoint>(_parent)
{
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
  gzerr << "not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  // set joint anchor
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
math::Vector3 PluginUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  gzerr << "not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
math::Angle PluginUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;
  gzerr << "not implemented\n";
  return result;
}

//////////////////////////////////////////////////
double PluginUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;
  gzerr << "not implemented\n";
  return result;
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetVelocity(unsigned int _index, double _angle)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
void PluginUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  gzerr << "not implemented\n";
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
  gzerr << "not implemented\n";
  return true;
}

//////////////////////////////////////////////////
double PluginUniversalJoint::GetParam(
  const std::string &_key, unsigned int _index)
{
  gzerr << "not implemented\n";
  return PluginJoint::GetParam(_key, _index);
}
