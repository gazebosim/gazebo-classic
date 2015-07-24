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

#include <string>
#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTJointPrivate.hh"
#include "gazebo/physics/dart/DARTScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTScrewJoint::DARTScrewJoint(BasePtr _parent)
  : ScrewJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTScrewJoint::~DARTScrewJoint()
{
}

//////////////////////////////////////////////////
void DARTScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<DARTJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);

  this->dataPtr->dtProperties.reset(
        new dart::dynamics::ScrewJoint::Properties(
          *this->dataPtr->dtProperties.get()));
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
math::Vector3 DARTScrewJoint::GetAnchor(unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<math::Vector3>(
          "Anchor" + std::to_string(_index));
  }

  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
      this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTScrewJoint::GetGlobalAxis(unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<math::Vector3>(
          "Axis" + std::to_string(_index));
  }

  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index < this->GetAngleCount())
  {
    dart::dynamics::ScrewJoint *dtScrewJoint =
        reinterpret_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtScrewJoint->getAxis();
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
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Axis" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetAxis, this, _index, _axis));
    return;
  }

  if (_index < this->GetAngleCount())
  {
    dart::dynamics::ScrewJoint *dtScrewJoint =
        reinterpret_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494
    // joint-axis-reference-frame-doesnt-match
    Eigen::Vector3d dartVec3 = DARTTypes::ConvVec3(_axis);
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    dartVec3 = dartTransfJointLeftToParentLink.linear() * dartVec3;

    dtScrewJoint->setAxis(dartVec3);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetVelocity(unsigned int _index) const
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
    gzerr << "DARTScrewJoint::GetVelocity: Not implemented for index[1].\n";
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Velocity" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetVelocity, this, _index, _vel));
    return;
  }

  if (_index == 0)
    this->dataPtr->dtJoint->setVelocity(0, _vel);
  else if (_index == 1)
    gzerr << "DARTScrewJoint::SetVelocity: Not implemented for index[1].\n";
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

  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "ThreadPitch",
        boost::bind(&DARTScrewJoint::SetThreadPitch, this, _threadPitch));
    return;
  }

  dart::dynamics::ScrewJoint *dtScrewJoint =
      reinterpret_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

  dtScrewJoint->setPitch(DARTTypes::ConvPitch(_threadPitch));
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
  GZ_ASSERT(
    !this->dataPtr->IsInitialized() ||
    (std::abs(reinterpret_cast<dart::dynamics::ScrewJoint *>(
      this->dataPtr->dtJoint)->getPitch() -
      DARTTypes::ConvPitch(this->threadPitch)) < 1e-6),
    "Gazebo and DART disagree in thread pitch.");

  return this->threadPitch;
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "Invalid index[" << _index << "]\n";
    return false;
  }

  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<double>(_key + std::to_string(_index));

  if (_key  == "thread_pitch")
  {
    return this->GetThreadPitch();
  }
  else if (_key == "friction")
  {
    if (_index == 0)
    {
      return this->dataPtr->dtJoint->getCoulombFriction(_index);
    }
    else if (_index == 1)
    {
      gzerr << "DARTScrewJoint::GetParam(friction): "
            << "Not implemented for index[1].\n";
      return false;
    }
    else
    {
      gzerr << "Should never be here. Joint index invalid limit not set.\n";
      return false;
    }
  }

  return DARTJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
bool DARTScrewJoint::SetParam(const std::string &_key,
                              unsigned int _index,
                              const boost::any &_value)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "Invalid index[" << _index << "]\n";
    return false;
  }

  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          _key + std::to_string(_index),
          boost::bind(&DARTScrewJoint::SetParam, this, _key, _index, _value));
    return true;
  }

  // try because boost::any_cast can throw
  try
  {
    if (_key  == "thread_pitch")
    {
      this->SetThreadPitch(boost::any_cast<double>(_value));
      return true;
    }
    else if (_key == "friction")
    {
      if (_index == 0)
      {
        this->dataPtr->dtJoint->setCoulombFriction(
              _index, boost::any_cast<double>(_value));
        return true;
      }
      else if (_index == 1)
      {
        gzerr << "DARTScrewJoint::SetParam(friction): "
              << "Not implemented for index[1].\n";
        return false;
      }
      else
      {
        gzerr << "Should never be here. Joint index invalid limit not set.\n";
        return false;
      }
    }
  }
  catch(const boost::bad_any_cast &_e)
  {
    gzerr << "SetParam(" << _key << ")"
          << " boost any_cast error:" << _e.what()
          << std::endl;
    return false;
  }

  return DARTJoint::SetParam(_key, _index, _value);
}

//////////////////////////////////////////////////
math::Angle DARTScrewJoint::GetAngleImpl(unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<math::Angle>("Angle");

  math::Angle result;

  if (_index == 0)
  {
    // angular position
    const double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result.SetFromRadian(radianAngle);
  }
  else if (_index == 1)
  {
    dart::dynamics::ScrewJoint *dtScrewJoint =
        reinterpret_cast<dart::dynamics::ScrewJoint *>(
          this->dataPtr->dtJoint);

    // linear position
    const double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result.SetFromRadian(-radianAngle /
                         DARTTypes::ConvPitch(dtScrewJoint->getPitch()));
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetMaxForce(unsigned int _index, double _force)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "MaxForce" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetMaxForce, this, _index, _force));
    return;
  }

  if (_index == 0)
  {
    this->dataPtr->dtJoint->setForceLowerLimit(0, -_force);
    this->dataPtr->dtJoint->setForceUpperLimit(0, _force);
  }
  else if (_index == 1)
  {
    gzerr << "DARTScrewJoint::SetMaxForce: Not implemented for index[1].\n";
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetMaxForce(unsigned int _index)
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "MaxForce" + std::to_string(_index));
  }

  double result = 0.0;

  if (_index == 0)
  {
    // Assume that the lower limit and upper limit has equal magnitute
    // result = this->dataPtr->dtJoint->getForceLowerLimit(0);
    result = this->dataPtr->dtJoint->getForceUpperLimit(0);
  }
  else if (_index == 1)
  {
    gzerr << "DARTScrewJoint::GetMaxForce: Not implemented for index[1].\n";
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
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Force" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetForceImpl, this, _index, _effort));
    return;
  }

  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else if (_index == 1)
    gzerr << "DARTScrewJoint::SetForceImpl: Not implemented for index[1].\n";
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
math::Angle DARTScrewJoint::GetHighStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
    if (!this->dataPtr->IsInitialized())
    {
      return this->dataPtr->GetCached<math::Angle>(
            "HighStop" + std::to_string(_index));
    }

    return this->dataPtr->dtJoint->getPositionUpperLimit(0);
  case 1:
    gzerr << "DARTScrewJoint::GetHighStop: Not implemented for index[1].\n";
    break;
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
    if (!this->dataPtr->IsInitialized())
    {
      return this->dataPtr->GetCached<math::Angle>(
            "LowStop" + std::to_string(_index));
    }

    return this->dataPtr->dtJoint->getPositionLowerLimit(0);
  case 1:
    gzerr << "DARTScrewJoint::GetLowStop: Not implemented for index[1].\n";
    break;
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return math::Angle();
}
