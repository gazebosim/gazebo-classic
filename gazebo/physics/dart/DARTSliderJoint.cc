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

#include <boost/bind.hpp>
#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTJointPrivate.hh"
#include "gazebo/physics/dart/DARTSliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTSliderJoint::DARTSliderJoint(BasePtr _parent)
  : SliderJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTSliderJoint::~DARTSliderJoint()
{
}

//////////////////////////////////////////////////
void DARTSliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<DARTJoint>::Load(_sdf);

  this->dataPtr->dtProperties.reset(
        new dart::dynamics::PrismaticJoint::Properties(
          *this->dataPtr->dtProperties.get()));
}

//////////////////////////////////////////////////
void DARTSliderJoint::Init()
{
  SliderJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTSliderJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Axis" + std::to_string(_index));
  }

  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
  {
    dart::dynamics::PrismaticJoint *dtPrismaticJoint =
        dynamic_cast<dart::dynamics::PrismaticJoint *>(
          this->dataPtr->dtJoint);
    GZ_ASSERT(dtPrismaticJoint, "PrismaticJoint is NULL");

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtPrismaticJoint->getAxis();
    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return DARTTypes::ConvVec3Ign(globalAxis);
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Axis" + std::to_string(_index),
          boost::bind(&DARTSliderJoint::SetAxis, this, _index, _axis));
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
  {
    dart::dynamics::PrismaticJoint *dtPrismaticJoint =
        dynamic_cast<dart::dynamics::PrismaticJoint *>(
          this->dataPtr->dtJoint);
    GZ_ASSERT(dtPrismaticJoint, "PrismaticJoint is NULL");

    Eigen::Vector3d dartVec3 = DARTTypes::ConvVec3(
        this->AxisFrameOffset(0).RotateVector(_axis));
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    dartVec3 = dartTransfJointLeftToParentLink.linear() * dartVec3;

    dtPrismaticJoint->setAxis(dartVec3);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}
