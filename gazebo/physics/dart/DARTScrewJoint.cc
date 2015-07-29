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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTScrewJoint::DARTScrewJoint(BasePtr _parent)
    : ScrewJoint<DARTJoint>(_parent),
      dartScrewJoint(new dart::dynamics::ScrewJoint())
{
  this->dtJoint = dartScrewJoint;
}

//////////////////////////////////////////////////
DARTScrewJoint::~DARTScrewJoint()
{
  delete dartScrewJoint;
}

//////////////////////////////////////////////////
void DARTScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<DARTJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &/*_anchor*/)
{
  gzerr << "DARTScrewJoint::SetAnchor not implemented.\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::Init()
{
  ScrewJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 DARTScrewJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dtChildBodyNode->getTransform() *
                        this->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTScrewJoint::GetGlobalAxis(unsigned int _index) const
{
  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index < this->GetAngleCount())
  {
    Eigen::Isometry3d T = this->dtChildBodyNode->getTransform() *
                          this->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = this->dartScrewJoint->getAxis();
    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494
  // joint-axis-reference-frame-doesnt-match
  return DARTTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (_index == 0)
  {
    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494
    // joint-axis-reference-frame-doesnt-match
    Eigen::Vector3d dartVec3 = DARTTypes::ConvVec3(_axis);
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dtJoint->getTransformFromParentBodyNode().inverse();
    dartVec3 = dartTransfJointLeftToParentLink.linear() * dartVec3;

    this->dartScrewJoint->setAxis(dartVec3);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (_index == 0)
    result = this->dtJoint->getVelocity(0);
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (_index == 0)
    this->dtJoint->setVelocity(0, _vel);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetThreadPitch(unsigned int _index, double _threadPitch)
{
  if (_index >= this->GetAngleCount())
    gzerr << "Invalid index[" << _index << "]\n";

  this->SetThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetThreadPitch(double _threadPitch)
{
  this->threadPitch = _threadPitch;

  this->dartScrewJoint->setPitch(DARTTypes::InvertThreadPitch(_threadPitch));
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetThreadPitch(unsigned int _index)
{
  if (_index >= this->GetAngleCount())
    gzerr << "Invalid index[" << _index << "]\n";

  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetThreadPitch()
{
  double result = this->threadPitch;

  if (this->dartScrewJoint)
    result = DARTTypes::InvertThreadPitch(this->dartScrewJoint->getPitch());
  else
    gzwarn << "dartScrewJoint not created yet, returning cached threadPitch.\n";

  return result;
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->GetThreadPitch();
  else
    return DARTJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
math::Angle DARTScrewJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (this->dartScrewJoint)
  {
    if (_index == 0)
    {
      // angular position
      result.SetFromRadian(this->dartScrewJoint->getPosition(0));
    }
    else if (_index == 1)
    {
      // linear position
      const double radianAngle = this->dartScrewJoint->getPosition(0);
      result.SetFromRadian(-radianAngle /
                           DARTTypes::InvertThreadPitch(dartScrewJoint->getPitch()));
    }
    else
    {
      gzerr << "Invalid index[" << _index << "]\n";
    }
  }
  else
  {
    gzerr << "dartScrewJoint not created yet\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetMaxForce(unsigned int _index, double _force)
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
double DARTScrewJoint::GetMaxForce(unsigned int _index)
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
void DARTScrewJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dtJoint->setForce(0, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
math::Angle DARTScrewJoint::GetHighStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
    return this->dtJoint->getPositionUpperLimit(0);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle DARTScrewJoint::GetLowStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
    return this->dtJoint->getPositionLowerLimit(0);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return math::Angle();
}
