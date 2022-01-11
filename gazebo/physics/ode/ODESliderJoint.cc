/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODESliderJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODESliderJoint::ODESliderJoint(dWorldID _worldId, BasePtr _parent)
    : SliderJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateSlider(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODESliderJoint::~ODESliderJoint()
{
  this->applyDamping.reset();
}

//////////////////////////////////////////////////
void ODESliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODESliderJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetSliderAxis(this->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
double ODESliderJoint::PositionImpl(const unsigned int /*_index*/) const
{
  double result = ignition::math::NAN_D;
  if (this->jointId)
    result = dJointGetSliderPosition(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODESliderJoint::GetVelocity(unsigned int /*index*/) const
{
  double result = 0;
  if (this->jointId)
    result = dJointGetSliderPositionRate(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODESliderJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void ODESliderJoint::SetAxis(const unsigned int /*index*/,
                             const ignition::math::Vector3d &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  // ODE needs global axis
  auto globalAxis = this->WorldPose().Rot() * _axis;

  if (this->jointId)
  {
    dJointSetSliderAxis(this->jointId,
                        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODESliderJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
{
  if (this->jointId)
    dJointAddSliderForce(this->jointId, _effort);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODESliderJoint::SetParam(unsigned int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  dJointSetSliderParam(this->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double ODESliderJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetSliderParam(this->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODESliderJoint::Anchor(
    const unsigned int /*_index*/) const
{
  gzlog << "ODESliderJoint::Anchor not implemented.\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void ODESliderJoint::SetAnchor(const unsigned int /*_index*/,
  const ignition::math::Vector3d &/*_anchor*/)
{
  gzlog << "ODESliderJoint::SetAnchor not implemented.\n";
}
