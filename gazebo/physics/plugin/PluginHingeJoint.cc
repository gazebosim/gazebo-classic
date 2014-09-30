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
/* Desc: A PluginHingeJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginHingeJoint::PluginHingeJoint(BasePtr _parent)
    : HingeJoint<PluginJoint>(_parent)
{
}

//////////////////////////////////////////////////
PluginHingeJoint::~PluginHingeJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
void PluginHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<PluginJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 PluginHingeJoint::GetAnchor(unsigned int /*index*/) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginHingeJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}


//////////////////////////////////////////////////
math::Vector3 PluginHingeJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginHingeJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  PluginJoint::SetAxis(_index, _axis);

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  // Plugin needs global axis
  math::Quaternion axisFrame = this->GetAxisFrame(0);
  math::Vector3 globalAxis = axisFrame.RotateVector(_axis);
}

//////////////////////////////////////////////////
math::Angle PluginHingeJoint::GetAngleImpl(unsigned int /*index*/) const
{
  math::Angle result;
  return result;
}

//////////////////////////////////////////////////
double PluginHingeJoint::GetVelocity(unsigned int /*index*/) const
{
  double result = 0;

  return result;
}

//////////////////////////////////////////////////
void PluginHingeJoint::SetVelocity(unsigned int /*index*/, double _angle)
{
  /// \TODO: FIXME: change this implementation to kinematic velocity setting
  /// similar to BulletHingeJoint::SetVelocity
}

//////////////////////////////////////////////////
void PluginHingeJoint::SetMaxForce(unsigned int /*index*/, double _t)
{
}

//////////////////////////////////////////////////
double PluginHingeJoint::GetMaxForce(unsigned int /*index*/)
{
  return 0;
}

//////////////////////////////////////////////////
void PluginHingeJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
{
}

//////////////////////////////////////////////////
double PluginHingeJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  return result;
}

//////////////////////////////////////////////////
void PluginHingeJoint::SetParam(unsigned int _parameter, double _value)
{
  PluginJoint::SetParam(_parameter, _value);
}
