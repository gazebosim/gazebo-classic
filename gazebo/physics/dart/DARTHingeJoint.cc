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
  : HingeJoint<DARTJoint>(_parent),
    dartRevJoint(new dart::dynamics::RevoluteJoint())
{
  this->dartJoint = this->dartRevJoint;
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
  HingeJoint<DARTJoint>::Init();
  this->dartRevJoint->setDampingCoefficient(0, dampingCoefficient);
}

//////////////////////////////////////////////////
math::Vector3 DARTHingeJoint::GetAnchor(int /*index*/) const
{
  return DARTTypes::ConvVec3(this->dartRevJoint->getWorldOrigin());
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetAnchor(int /*index*/, const math::Vector3& /*_anchor*/)
{
  // We do not do anything here because DART does not store the positon
  // of the joint.
}

//////////////////////////////////////////////////
math::Vector3 DARTHingeJoint::GetGlobalAxis(int /*_index*/) const
{
  // Axis in local frame of this joint
  Eigen::Vector3d globalAxis = dartRevJoint->getWorldAxis();

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  return DARTTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetAxis(int /*index*/, const math::Vector3& _axis)
{
  Eigen::Vector3d dartAxis = DARTTypes::ConvVec3(_axis);

  //----------------------------------------------------------------------------
  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  Eigen::Isometry3d dartTransfJointLeftToParentLink
      = dartRevJoint->getTransformFromParentBodyNode().inverse();
  dartAxis = dartTransfJointLeftToParentLink.linear() * dartAxis;
  //----------------------------------------------------------------------------

  dartRevJoint->setAxis(dartAxis);
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetDamping(int _index, double _damping)
{
  assert(_index == 0);
  assert(_damping >= 0.0);

  this->dampingCoefficient = _damping;
  dartRevJoint->setDampingCoefficient(0, _damping);
}

//////////////////////////////////////////////////
math::Angle DARTHingeJoint::GetAngleImpl(int /*index*/) const
{
  math::Angle result;

  assert(this->dartRevJoint);
  assert(this->dartRevJoint->getDOF() == 1);

  // Hinge joint has only one dof.
  double radianAngle = this->dartRevJoint->getGenCoord(0)->get_q();
  result.SetFromRadian(radianAngle);

  return result;
}

//////////////////////////////////////////////////
double DARTHingeJoint::GetVelocity(int /*index*/) const
{
  return this->dartRevJoint->getGenCoord(0)->get_dq();
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

  dartJoint->getGenCoord(0)->set_tau(_torque);
}
