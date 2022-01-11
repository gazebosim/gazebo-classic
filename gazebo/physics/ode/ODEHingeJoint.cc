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
/* Desc: A ODEHingeJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEHingeJoint::ODEHingeJoint(dWorldID _worldId, BasePtr _parent)
    : HingeJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateHinge(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODEHingeJoint::~ODEHingeJoint()
{
  this->applyDamping.reset();
}

//////////////////////////////////////////////////
void ODEHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEHingeJoint::Anchor(
    const unsigned int /*index*/) const
{
  dVector3 result;

  if (this->jointId)
    dJointGetHingeAnchor(this->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetAnchor(const unsigned int /*index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  if (this->jointId)
    dJointSetHingeAnchor(this->jointId, _anchor.X(), _anchor.Y(), _anchor.Z());
  else
    gzerr << "ODE Joint ID is invalid\n";
}


//////////////////////////////////////////////////
ignition::math::Vector3d ODEHingeJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetHingeAxis(this->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetAxis(const unsigned int _index,
                            const ignition::math::Vector3d &_axis)
{
  ODEJoint::SetAxis(_index, _axis);

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  // ODE needs global axis
  auto globalAxis = this->WorldPose().Rot() * _axis;

  if (this->jointId)
  {
    dJointSetHingeAxis(this->jointId, globalAxis.X(), globalAxis.Y(),
        globalAxis.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEHingeJoint::PositionImpl(const unsigned int /*index*/) const
{
  double result = ignition::math::NAN_D;
  if (this->jointId)
    result = dJointGetHingeAngle(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEHingeJoint::GetVelocity(unsigned int /*index*/) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetHingeAngleRate(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
{
  if (this->jointId)
    dJointAddHingeTorque(this->jointId, _effort);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEHingeJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetHingeParam(this->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetParam(unsigned int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->jointId)
    dJointSetHingeParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetCumulativeAngle(double _angle)
{
  dJointSetHingeCumulativeAngle(this->jointId, _angle);
}
