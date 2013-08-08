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
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTHingeJoint.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTHingeJoint::DARTHingeJoint(BasePtr _parent)
  : HingeJoint<DARTJoint>(_parent)
{
  this->dartJoint = new dart::dynamics::RevoluteJoint();
}

//////////////////////////////////////////////////
DARTHingeJoint::~DARTHingeJoint()
{
}

//////////////////////////////////////////////////
void DARTHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<DARTJoint>::Load(_sdf);

}

//////////////////////////////////////////////////
void DARTHingeJoint::Init()
{
  // Create dart joint first.
  //this->dartJoint = new dart::dynamics::RevoluteJoint();
  HingeJoint<DARTJoint>::Init();
  this->dartJoint->setDampingCoefficient(0, dampingCoefficient);
}

//////////////////////////////////////////////////
math::Vector3 DARTHingeJoint::GetAnchor(int /*index*/) const
{
  math::Vector3 result;

  // setting anchor relative to gazebo link frame pose
  if (this->childLink)
    result = poseChildLinkToJoint.pos + this->childLink->GetWorldPose().pos;
  else
    result = math::Vector3(0, 0, 0);

  return result;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetAnchor(int /*index*/, const math::Vector3& /*_anchor*/)
{
  // TODO: We do not do anything here because DART does not store the positon
  // of the joint.
}

//////////////////////////////////////////////////
math::Vector3 DARTHingeJoint::GetGlobalAxis(int /*_index*/) const
{
  // Axis in local frame of this joint
  dart::dynamics::RevoluteJoint* dartRevJoint
      = dynamic_cast<dart::dynamics::RevoluteJoint*>(this->dartJoint);
  dart::math::Axis globalAxis = dartRevJoint->getAxisGlobal();

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  return DARTUtils::ConvertAxis(globalAxis);
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetAxis(int /*index*/, const math::Vector3& _axis)
{
  dart::dynamics::RevoluteJoint* dartRevJoint
      = dynamic_cast<dart::dynamics::RevoluteJoint*>(this->dartJoint);

  dart::math::Axis dartAxis = DARTUtils::ConvertAxis(_axis);

  //----------------------------------------------------------------------------
  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  dart::math::SE3 dartTransfJointLeftToParentLink
      = dart::math::Inv(dartRevJoint->getLocalTransformationFromParentBody());
  dartAxis = dart::math::Rotate(dartTransfJointLeftToParentLink, dartAxis);
  //----------------------------------------------------------------------------

  dartRevJoint->setAxis(dartAxis);
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetDamping(int _index, double _damping)
{
  assert(_index == 0);
  assert(_damping >= 0.0);

  dart::dynamics::RevoluteJoint* dartRevJoint
      = dynamic_cast<dart::dynamics::RevoluteJoint*>(this->dartJoint);

  this->dampingCoefficient = _damping;
  dartRevJoint->setDampingCoefficient(0, _damping);
}

//////////////////////////////////////////////////
math::Angle DARTHingeJoint::GetAngleImpl(int /*index*/) const
{
  math::Angle result;

  assert(this->dartJoint);
  assert(this->dartJoint->getDOF() == 1);

  // Hinge joint has only one dof.
  double radianAngle = this->dartJoint->getDof(0)->get_q();
  result.SetFromRadian(radianAngle);

  return result;
}

//////////////////////////////////////////////////
double DARTHingeJoint::GetVelocity(int /*index*/) const
{
  double result;

  result = this->dartJoint->getDof(0)->get_dq();

  return result;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetVelocity(int /*index*/, double /*_vel*/)
{
  // TODO: Do nothing because DART accept only torques (forces) of joint as
  // input.
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetMaxForce(int /*index*/, double /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTHingeJoint::GetMaxForce(int /*index*/)
{
  gzwarn << "Not implemented!\n";
  return 0.0;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetForce(int _index, double _torque)
{
  DARTJoint::SetForce(_index, _torque);

  dartJoint->getDof(0)->set_tau(_torque);
}
