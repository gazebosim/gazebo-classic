/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
  this->dataPtr->dtJoint = new dart::dynamics::UniversalJoint();
}

//////////////////////////////////////////////////
DARTUniversalJoint::~DARTUniversalJoint()
{
  delete this->dataPtr->dtJoint;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<DARTJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::Init()
{
  UniversalJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
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

  return DARTTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetAxis(unsigned int _index,
    const math::Vector3 &_axis)
{
  Eigen::Vector3d dtAxis = DARTTypes::ConvVec3(
      this->GetAxisFrameOffset(_index).RotateVector(_axis));
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
math::Angle DARTUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (_index == 0)
  {
    double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result.SetFromRadian(radianAngle);
  }
  else if (_index == 1)
  {
    double radianAngle = this->dataPtr->dtJoint->getPosition(1);
    result.SetFromRadian(radianAngle);
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
  if (_index < this->GetAngleCount())
  {
    this->dataPtr->dtJoint->setVelocity(_index, _vel);
    this->dataPtr->dtJoint->getSkeleton()->computeForwardKinematics(
          false, true, false);
  }
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetMaxForce(unsigned int _index, double _force)
{
  if (_index == 0)
  {
    this->dataPtr->dtJoint->setForceLowerLimit(0, -_force);
    this->dataPtr->dtJoint->setForceUpperLimit(0, _force);
  }
  else if (_index == 1)
  {
    this->dataPtr->dtJoint->setForceLowerLimit(1, -_force);
    this->dataPtr->dtJoint->setForceUpperLimit(1, _force);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetMaxForce(unsigned int _index)
{
  double result = 0.0;

  if (_index == 0)
  {
    // Assume that the lower limit and upper limit has equal magnitute
    // result = this->dataPtr->dtJoint->getForceLowerLimit(0);
    result = this->dataPtr->dtJoint->getForceUpperLimit(0);
  }
  else if (_index == 1)
  {
    // Assume that the lower limit and upper limit has equal magnitute
    // result = this->dataPtr->dtJoint->getForceLowerLimit(1);
    result = this->dataPtr->dtJoint->getForceUpperLimit(1);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else if (_index == 1)
    this->dataPtr->dtJoint->setForce(1, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}
