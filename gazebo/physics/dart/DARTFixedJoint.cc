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
#include "gazebo/physics/dart/DARTFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTFixedJoint::DARTFixedJoint(BasePtr _parent)
  : FixedJoint<DARTJoint>(_parent),
    dtWeldJoint(new dart::dynamics::WeldJoint())
{
  this->dtJoint = this->dtWeldJoint;
}

//////////////////////////////////////////////////
DARTFixedJoint::~DARTFixedJoint()
{
  delete dtWeldJoint;
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
  Eigen::Isometry3d T = this->dtChildBodyNode->getTransform() *
                        this->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "called invalid method GetGlobalAxis in a fixed joint\n";

  return math::Vector3();
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetAxis(unsigned int /*_index*/,
                             const math::Vector3& /*_axis*/)
{
  gzerr << "called invalid method SetAxis in a fixed joint\n";

  return;
}

//////////////////////////////////////////////////
math::Angle DARTFixedJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  math::Angle result;

  gzerr << "called invalid method GetAngleImpl in a fixed joint\n";

  return result;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_vel*/)
{
  gzerr << "called invalid method SetVelocity in a fixed joint\n";

  return;
}

//////////////////////////////////////////////////
double DARTFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  double result = 0.0;

  gzerr << "called invalid method SetVelocity in a fixed joint\n";

  return result;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetMaxForce(unsigned int /*_index*/, double /*_force*/)
{
  gzerr << "called invalid method SetMaxForce in a fixed joint\n";

  return;
}

//////////////////////////////////////////////////
double DARTFixedJoint::GetMaxForce(unsigned int /*_index*/)
{
  double result = 0.0;

  gzerr << "called invalid method GetMaxForce in a fixed joint\n";

  return result;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzerr << "called invalid method SetForceImpl in a fixed joint\n";

  return;
}
