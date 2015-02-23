/*
 * Copyright 2014 Open Source Robotics Foundation
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
math::Vector3 DARTBallJoint::GetAnchor(int /*_index*/) const
{
  Eigen::Isometry3d T = this->dtChildBodyNode->getWorldTransform() *
                        this->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}
