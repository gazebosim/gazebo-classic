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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTBallJoint::DARTBallJoint(BasePtr _parent)
  : BallJoint<DARTJoint>(_parent),
    dtBallJoint(new dart::dynamics::BallJoint())
{
  this->dtJoint = this->dtBallJoint;
}

//////////////////////////////////////////////////
DARTBallJoint::~DARTBallJoint()
{
  delete dtBallJoint;
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
math::Vector3 DARTBallJoint::GetAnchor(unsigned int /*_index*/) const
{
  Eigen::Isometry3d T = this->dtChildBodyNode->getTransform() *
                        this->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTBallJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  return math::Vector3();
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
double DARTBallJoint::GetMaxForce(unsigned int /*_index*/)
{
  return 0;
}

//////////////////////////////////////////////////
void DARTBallJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
}

//////////////////////////////////////////////////
math::Angle DARTBallJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "DARTBallJoint::GetAngleImpl not implemented" << std::endl;
  return math::Angle(0);
}

//////////////////////////////////////////////////
void DARTBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "DARTBallJoint::SetForceImpl not implemented";
}

//////////////////////////////////////////////////
void DARTBallJoint::SetAxis(unsigned int /*_index*/,
                            const math::Vector3 &/*_axis*/)
{
  gzerr << "DARTBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
math::Angle DARTBallJoint::GetHighStop(unsigned int /*_index*/)
{
  gzerr << "DARTBallJoint::GetHighStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle DARTBallJoint::GetLowStop(unsigned int /*_index*/)
{
  gzerr << "DARTBallJoint::GetLowStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
bool DARTBallJoint::SetHighStop(unsigned int /*_index*/,
                                const math::Angle &/*_angle*/)
{
  gzerr << "DARTBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool DARTBallJoint::SetLowStop(unsigned int /*_index*/,
                               const math::Angle &/*_angle*/)
{
  gzerr << "DARTBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}
