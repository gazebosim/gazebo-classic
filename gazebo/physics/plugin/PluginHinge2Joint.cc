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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginHinge2Joint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
PluginHinge2Joint::PluginHinge2Joint(BasePtr _parent)
    : Hinge2Joint<PluginJoint>(_parent)
{
}

//////////////////////////////////////////////////
PluginHinge2Joint::~PluginHinge2Joint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
void PluginHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<PluginJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 PluginHinge2Joint::GetAnchor(unsigned int _index) const
{
  double result[3];

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginHinge2Joint::SetAnchor(unsigned int /*_index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void PluginHinge2Joint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// Plugin needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  math::Vector3 globalAxis = _axis;
  if (this->parentLink)
    globalAxis =
      this->GetParent()->GetModel()->GetWorldPose().rot.RotateVector(_axis);

  if (this->jointId)
  {
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Vector3 PluginHinge2Joint::GetGlobalAxis(unsigned int _index) const
{
  double result[3];

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
math::Angle PluginHinge2Joint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (this->jointId)
  {
    if (_index == 0)
      result = 0;
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double PluginHinge2Joint::GetVelocity(unsigned int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    if (_index == 0)
      result = 0;
    else
      result = 0;
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginHinge2Joint::SetVelocity(unsigned int _index, double _angle)
{
}

//////////////////////////////////////////////////
double PluginHinge2Joint::GetMaxForce(unsigned int _index)
{
}


//////////////////////////////////////////////////
void PluginHinge2Joint::SetMaxForce(unsigned int _index, double _t)
{
}


//////////////////////////////////////////////////
void PluginHinge2Joint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->jointId)
  {
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double PluginHinge2Joint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = 0;
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginHinge2Joint::SetParam(unsigned int _parameter, double _value)
{
  PluginJoint::SetParam(_parameter, _value);
}
