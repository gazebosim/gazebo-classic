/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/physics/dart/DARTJointPrivate.hh"
#include "gazebo/physics/dart/DARTFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTFixedJoint::DARTFixedJoint(BasePtr _parent)
  : FixedJoint<DARTJoint>(_parent)
{
  this->dataPtr->dtJoint = new dart::dynamics::WeldJoint();
}

//////////////////////////////////////////////////
DARTFixedJoint::~DARTFixedJoint()
{
  delete this->dataPtr->dtJoint;
}

//////////////////////////////////////////////////
void DARTFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<DARTJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTFixedJoint::Init()
{
  FixedJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 DARTFixedJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzwarn << "DARTFixedJoint: called method "
         << "GetGlobalAxis that is not valid for joints of type fixed.\n";

  return math::Vector3();
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetAxis(unsigned int /*_index*/,
                             const math::Vector3& /*_axis*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
math::Angle DARTFixedJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzwarn << "DARTFixedJoint: called method "
         << "GetAngleImpl that is not valid for joints of type fixed.\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_vel*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
double DARTFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzwarn << "DARTFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetMaxForce(unsigned int /*_index*/, double /*_force*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetMaxForce that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double DARTFixedJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "GetMaxForce that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
}
