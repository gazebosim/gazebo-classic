/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTSliderJoint::DARTSliderJoint(BasePtr _parent)
    : SliderJoint<DARTJoint>(_parent),
      dartPrismaticJoint(new dart::dynamics::PrismaticJoint())
{
  this->dartJoint = this->dartPrismaticJoint;
}

//////////////////////////////////////////////////
DARTSliderJoint::~DARTSliderJoint()
{
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
  this->dartPrismaticJoint->setDampingCoefficient(0, dampingCoefficient);
}

//////////////////////////////////////////////////
math::Vector3 DARTSliderJoint::GetAnchor(int /*_index*/) const
{
  return DARTTypes::ConvVec3(this->dartPrismaticJoint->getWorldOrigin());
}

//////////////////////////////////////////////////
math::Vector3 DARTSliderJoint::GetGlobalAxis(int /*_index*/) const
{
  Eigen::Vector3d globalAxis = this->dartPrismaticJoint->getWorldAxis();

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  return DARTTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
  Eigen::Vector3d dartVec3 = DARTTypes::ConvVec3(_axis);

  //----------------------------------------------------------------------------
  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  Eigen::Isometry3d dartTransfJointLeftToParentLink
      = this->dartPrismaticJoint->getTransformFromParentBodyNode().inverse();
  dartVec3 = dartTransfJointLeftToParentLink.linear() * dartVec3;
  //----------------------------------------------------------------------------

  this->dartPrismaticJoint->setAxis(dartVec3);
}

//////////////////////////////////////////////////
math::Angle DARTSliderJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;

  assert(this->dartJoint);
  assert(this->dartJoint->getNumGenCoords() == 1);

  // Hinge joint has only one dof.
  double radianAngle = this->dartJoint->getGenCoord(0)->get_q();
  result.SetFromRadian(radianAngle);

  return result;
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetVelocity(int /*index*/) const
{
  return this->dartJoint->getGenCoord(0)->get_dq();
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetForce(int /*index*/, double _force)
{
  this->dartJoint->getGenCoord(0)->set_tauMax(_force);
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetMaxForce(int /*_index*/)
{
  return this->dartJoint->getGenCoord(0)->get_tauMax();
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetMaxForce(int _index, double _torque)
{
  DARTJoint::SetForce(_index, _torque);

  this->dartJoint->getGenCoord(0)->set_tau(_torque);
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetForceImpl(int /*_index*/, double _effort)
{
  if (this->dartJoint)
  {
    this->dartJoint->getGenCoord(0)->set_tau(_effort);
  }
  else
  {
    gzerr << "DART revolute joint is invalid\n";
  }
}



