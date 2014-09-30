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
/* Desc: A slider or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */
#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginSliderJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
PluginSliderJoint::PluginSliderJoint(dWorldID _worldId, BasePtr _parent)
    : SliderJoint<PluginJoint>(_parent)
{
  this->jointId = dJointCreateSlider(_worldId, NULL);
}

//////////////////////////////////////////////////
PluginSliderJoint::~PluginSliderJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
void PluginSliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<PluginJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 PluginSliderJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetSliderAxis(this->jointId, result);
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
math::Angle PluginSliderJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  math::Angle result;
  if (this->jointId)
    result = dJointGetSliderPosition(this->jointId);
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double PluginSliderJoint::GetVelocity(unsigned int /*index*/) const
{
  double result = 0;
  if (this->jointId)
    result = dJointGetSliderPositionRate(this->jointId);
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginSliderJoint::SetVelocity(unsigned int /*index*/, double _angle)
{
  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void PluginSliderJoint::SetAxis(unsigned int /*index*/, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  // Plugin needs global axis
  math::Quaternion axisFrame = this->GetAxisFrame(0);
  math::Vector3 globalAxis = axisFrame.RotateVector(_axis);

  if (this->jointId)
  {
    dJointSetSliderAxis(this->jointId,
                        globalAxis.x, globalAxis.y, globalAxis.z);
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void PluginSliderJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
{
  if (this->jointId)
    dJointAddSliderForce(this->jointId, _effort);
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void PluginSliderJoint::SetParam(unsigned int _parameter, double _value)
{
  PluginJoint::SetParam(_parameter, _value);
  dJointSetSliderParam(this->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double PluginSliderJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetSliderParam(this->jointId, _parameter);
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginSliderJoint::SetMaxForce(unsigned int /*_index*/, double _t)
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double PluginSliderJoint::GetMaxForce(unsigned int /*_index*/)
{
  return this->GetParam(dParamFMax);
}

//////////////////////////////////////////////////
math::Vector3 PluginSliderJoint::GetAnchor(unsigned int /*_index*/) const
{
  dVector3 result;
  gzlog << "PluginSliderJoint::GetAnchor not implemented.\n";
  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginSliderJoint::SetAnchor(unsigned int /*_index*/,
  const math::Vector3 &/*_anchor*/)
{
  gzlog << "PluginSliderJoint::SetAnchor not implemented.\n";
}
