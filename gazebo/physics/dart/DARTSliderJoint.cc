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
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTSliderJoint.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTSliderJoint::DARTSliderJoint(BasePtr _parent)
    : SliderJoint<DARTJoint>(_parent)
{
  this->dartJoint = new dart::dynamics::PrismaticJoint();
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
  // Create dart joint first.
  //this->dartJoint = new dart::dynamics::PrismaticJoint();
  SliderJoint<DARTJoint>::Init();
  this->dartJoint->setDampingCoefficient(0, dampingCoefficient);
}

math::Vector3 DARTSliderJoint::GetAnchor(int /*_index*/) const
{
  math::Vector3 result;

  gzwarn << "Not implemented.\n";

  return result;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetAnchor(int /*_index*/, const math::Vector3& /*_anchor*/)
{
  gzwarn << "Not implemented.\n";
}

//////////////////////////////////////////////////
math::Vector3 DARTSliderJoint::GetGlobalAxis(int /*_index*/) const
{
  // Axis in local frame of this joint
  dart::dynamics::PrismaticJoint* dartPrismaticJoint
      = dynamic_cast<dart::dynamics::PrismaticJoint*>(this->dartJoint);
  Eigen::Vector3d globalAxis = dartPrismaticJoint->getAxisGlobal();

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  return DARTUtils::ConvertVector3(globalAxis);
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
  dart::dynamics::PrismaticJoint* dartPrismaticJoint
      = dynamic_cast<dart::dynamics::PrismaticJoint*>(this->dartJoint);

  Eigen::Vector3d dartVec3 = DARTUtils::ConvertVector3(_axis);

  //----------------------------------------------------------------------------
  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  Eigen::Isometry3d dartTransfJointLeftToParentLink
      = dartPrismaticJoint->getLocalTransformationFromParentBody().inverse();
  dartVec3 = dartTransfJointLeftToParentLink.rotation() * dartVec3;
  //----------------------------------------------------------------------------

  dartPrismaticJoint->setAxis(dartVec3);
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetDamping(int _index, double _damping)
{
  assert(_index == 0);
  assert(_damping >= 0.0);

  dart::dynamics::PrismaticJoint* dartPrismaticJoint
      = dynamic_cast<dart::dynamics::PrismaticJoint*>(this->dartJoint);

  this->dampingCoefficient = _damping;
  dartPrismaticJoint->setDampingCoefficient(0, _damping);
}

//////////////////////////////////////////////////
math::Angle DARTSliderJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;

  assert(this->dartJoint);
  assert(this->dartJoint->getDOF() == 1);

  // Hinge joint has only one dof.
  double radianAngle = this->dartJoint->getGenCoord(0)->get_q();
  result.SetFromRadian(radianAngle);

  return result;
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetVelocity(int /*index*/) const
{
  double result;

  result = this->dartJoint->getGenCoord(0)->get_dq();

  return result;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
  // TODO: Do nothing because DART accept only torques (forces) of joint as
  // input.
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetForce(int /*index*/, double /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetMaxForce(int /*_index*/)
{
  gzwarn << "Not implemented!\n";

  return 0;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetMaxForce(int _index, double _torque)
{
  DARTJoint::SetForce(_index, _torque);

  dartJoint->getGenCoord(0)->set_tau(_torque);
}





