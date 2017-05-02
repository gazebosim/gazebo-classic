/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTJointPrivate.hh"
#include "gazebo/physics/dart/DARTUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTUniversalJoint::DARTUniversalJoint(BasePtr _parent)
  : UniversalJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTUniversalJoint::~DARTUniversalJoint()
{
}

//////////////////////////////////////////////////
void DARTUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<DARTJoint>::Load(_sdf);

  this->dataPtr->dtProperties.reset(
        new dart::dynamics::UniversalJoint::Properties(
          *this->dataPtr->dtProperties.get()));
}

//////////////////////////////////////////////////
void DARTUniversalJoint::Init()
{
  UniversalJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTUniversalJoint::Anchor(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Anchor" + std::to_string(_index));
  }

  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3Ign(worldOrigin);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTUniversalJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Axis" + std::to_string(_index));
  }

  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index == 0)
  {
    dart::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getLocalTransform().inverse() *
        this->dataPtr->dtJoint->getTransformFromParentBodyNode();
    Eigen::Vector3d axis = dtUniveralJoint->getAxis1();

    globalAxis = T.linear() * axis;
  }
  else if (_index == 1)
  {
    dart::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtUniveralJoint->getAxis2();

    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return DARTTypes::ConvVec3Ign(globalAxis);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Axis" + std::to_string(_index),
          boost::bind(&DARTUniversalJoint::SetAxis, this, _index, _axis));
    return;
  }

  Eigen::Vector3d dtAxis = DARTTypes::ConvVec3(
      this->AxisFrameOffset(_index).RotateVector(_axis));
  Eigen::Isometry3d dtTransfJointLeftToParentLink
      = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
  dtAxis = dtTransfJointLeftToParentLink.linear() * dtAxis;

  if (_index == 0)
  {
    dart::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    dtUniveralJoint->setAxis1(dtAxis);
  }
  else if (_index == 1)
  {
    dart::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    dtUniveralJoint->setAxis2(dtAxis);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTUniversalJoint::PositionImpl(const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>("Angle" + std::to_string(_index));
  }

  double result = ignition::math::NAN_D;

  if (_index == 0)
  {
    result = this->dataPtr->dtJoint->getPosition(0);
  }
  else if (_index == 1)
  {
    result = this->dataPtr->dtJoint->getPosition(1);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetVelocity(unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "Velocity" + std::to_string(_index));
  }

  double result = 0.0;

  if (_index == 0)
    result = this->dataPtr->dtJoint->getVelocity(0);
  else if (_index == 1)
    result = this->dataPtr->dtJoint->getVelocity(1);
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Velocity" + std::to_string(_index),
          boost::bind(&DARTUniversalJoint::SetVelocity, this, _index, _vel),
          _vel);
    return;
  }

  if (_index < this->DOF())
    this->dataPtr->dtJoint->setVelocity(_index, _vel);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Force" + std::to_string(_index),
        boost::bind(&DARTUniversalJoint::SetForceImpl, this, _index, _effort));
    return;
  }

  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else if (_index == 1)
    this->dataPtr->dtJoint->setForce(1, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}
