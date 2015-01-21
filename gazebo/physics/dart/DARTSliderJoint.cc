/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTSliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTSliderJoint::DARTSliderJoint(BasePtr _parent)
    : SliderJoint<DARTJoint>(_parent),
      dtPrismaticJoint(new dart::dynamics::PrismaticJoint())
{
  this->dtJoint = this->dtPrismaticJoint;
}

//////////////////////////////////////////////////
DARTSliderJoint::~DARTSliderJoint()
{
  delete dtPrismaticJoint;
}

//////////////////////////////////////////////////
void DARTSliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<DARTJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTSliderJoint::Init()
{
  SliderJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 DARTSliderJoint::GetAnchor(unsigned int /*_index*/) const
{
  Eigen::Isometry3d T = this->dtChildBodyNode->getTransform() *
                        this->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTSliderJoint::GetGlobalAxis(unsigned int _index) const
{
  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index == 0)
  {
    Eigen::Isometry3d T = this->dtChildBodyNode->getTransform() *
                          this->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = this->dtPrismaticJoint->getAxis();
    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return DARTTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (_index == 0)
  {
    Eigen::Vector3d dartVec3 = DARTTypes::ConvVec3(
        this->GetAxisFrameOffset(0).RotateVector(_axis));
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dtJoint->getTransformFromParentBodyNode().inverse();
    dartVec3 = dartTransfJointLeftToParentLink.linear() * dartVec3;

    this->dtPrismaticJoint->setAxis(dartVec3);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
math::Angle DARTSliderJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (_index == 0)
  {
    double radianAngle = this->dtJoint->getPosition(0);
    result.SetFromRadian(radianAngle);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (_index == 0)
  {
    this->dtJoint->setVelocity(0, _vel);
    this->dtJoint->getSkeleton()->computeForwardKinematics(false, true, false);
  }
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (_index == 0)
    result = this->dtJoint->getVelocity(0);
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetMaxForce(unsigned int _index, double _force)
{
  if (_index == 0)
  {
    this->dtJoint->setForceLowerLimit(0, -_force);
    this->dtJoint->setForceUpperLimit(0, _force);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetMaxForce(unsigned int _index)
{
  double result = 0.0;

  if (_index == 0)
  {
    // Assume that the lower limit and upper limit has equal magnitute
    // result = this->dtJoint->getForceLowerLimit(0);
    result = this->dtJoint->getForceUpperLimit(0);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dtJoint->setForce(0, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}
