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
#include "gazebo/physics/dart/DARTBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTBallJoint::DARTBallJoint(BasePtr _parent)
  : BallJoint<DARTJoint>(_parent)
{
  this->dataPtr->dtProperties.reset(
      new dart::dynamics::BallJoint::Properties(
      *(this->dataPtr->dtProperties)));
}

//////////////////////////////////////////////////
DARTBallJoint::~DARTBallJoint()
{
}

//////////////////////////////////////////////////
void DARTBallJoint::Load(sdf::ElementPtr _sdf)
{
  BallJoint<DARTJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTBallJoint::Init()
{
  BallJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTBallJoint::Anchor(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Anchor" + std::to_string(_index));
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3Ign(worldOrigin);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTBallJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void DARTBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
}

//////////////////////////////////////////////////
double DARTBallJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "DARTBallJoint::GetVelocity not implemented" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
double DARTBallJoint::PositionImpl(const unsigned int /*_index*/) const
{
  gzerr << "DARTBallJoint::PositionImpl not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void DARTBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "DARTBallJoint::SetForceImpl not implemented";
}

//////////////////////////////////////////////////
void DARTBallJoint::SetAxis(const unsigned int /*_index*/,
                            const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "DARTBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
double DARTBallJoint::UpperLimit(const unsigned int /*_index*/) const
{
  gzerr << "DARTBallJoint::UpperLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double DARTBallJoint::LowerLimit(const unsigned int /*_index*/) const
{
  gzerr << "DARTBallJoint::LowerLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void DARTBallJoint::SetUpperLimit(const unsigned int /*_index*/,
                                  const double /*_limit*/)
{
  gzerr << "DARTBallJoint::SetUpperLimit not implemented" << std::endl;
}

//////////////////////////////////////////////////
void DARTBallJoint::SetLowerLimit(const unsigned int /*_index*/,
                                  const double /*_limit*/)
{
  gzerr << "DARTBallJoint::SetLowerLimit not implemented" << std::endl;
}
