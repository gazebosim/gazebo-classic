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
#include "gazebo/physics/dart/DARTHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTHinge2Joint::DARTHinge2Joint(BasePtr _parent)
  : Hinge2Joint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTHinge2Joint::~DARTHinge2Joint()
{
}

//////////////////////////////////////////////////
void DARTHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<DARTJoint>::Load(_sdf);

  this->dataPtr->dtProperties.reset(
        new dart::dynamics::UniversalJoint::Properties(
          *(this->dataPtr->dtProperties)));
}

//////////////////////////////////////////////////
void DARTHinge2Joint::Init()
{
  Hinge2Joint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Axis" + std::to_string(_index),
          boost::bind(&DARTHinge2Joint::SetAxis, this, _index, _axis));
    return;
  }

  Eigen::Vector3d dartAxis = DARTTypes::ConvVec3(_axis);

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
  {
    dart::dynamics::UniversalJoint *dtUniversalJoint =
        dynamic_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    GZ_ASSERT(dtUniversalJoint, "UniversalJoint is NULL");

    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    dartAxis = dartTransfJointLeftToParentLink.linear() * dartAxis;

    dtUniversalJoint->setAxis1(dartAxis);
  }
  else if (_index == 1)
  {
    dart::dynamics::UniversalJoint *dtUniversalJoint =
        dynamic_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    GZ_ASSERT(dtUniversalJoint, "UniversalJoint is NULL");
    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    dartAxis = dartTransfJointLeftToParentLink.linear() * dartAxis;

    dtUniversalJoint->setAxis2(dartAxis);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTHinge2Joint::GlobalAxis(
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
    dart::dynamics::UniversalJoint *dtUniversalJoint =
        dynamic_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    GZ_ASSERT(dtUniversalJoint, "UniversalJoint is NULL");

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getRelativeTransform().inverse() *
        this->dataPtr->dtJoint->getTransformFromParentBodyNode();
    Eigen::Vector3d axis = dtUniversalJoint->getAxis1();

    globalAxis = T.linear() * axis;
  }
  else if (_index == 1)
  {
    dart::dynamics::UniversalJoint *dtUniversalJoint =
        dynamic_cast<dart::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    GZ_ASSERT(dtUniversalJoint, "UniversalJoint is NULL");

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtUniversalJoint->getAxis2();

    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/
  // joint-axis-reference-frame-doesnt-match
  return DARTTypes::ConvVec3Ign(globalAxis);
}
