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
#include "gazebo/physics/dart/DARTUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTUniversalJoint::DARTUniversalJoint(BasePtr _parent)
    : UniversalJoint<DARTJoint>(_parent),
      dtUniveralJoint(new dart::dynamics::UniversalJoint())
{
  this->dtJoint = dtUniveralJoint;
}

//////////////////////////////////////////////////
DARTUniversalJoint::~DARTUniversalJoint()
{
  delete dtUniveralJoint;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<DARTJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::Init()
{
  UniversalJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dtChildBodyNode->getWorldTransform() *
                        this->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index == 0)
  {
    Eigen::Isometry3d T = this->dtChildBodyNode->getWorldTransform() *
                          this->dtJoint->getLocalTransform().inverse() *
                          this->dtJoint->getTransformFromParentBodyNode();
    Eigen::Vector3d axis = this->dtUniveralJoint->getAxis1();

    globalAxis = T.linear() * axis;
  }
  else if (_index == 1)
  {
    Eigen::Isometry3d T = this->dtChildBodyNode->getWorldTransform() *
                          this->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = this->dtUniveralJoint->getAxis2();

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
void DARTUniversalJoint::SetAxis(unsigned int _index,
    const math::Vector3 &_axis)
{
  Eigen::Vector3d dtAxis = DARTTypes::ConvVec3(_axis);

  if (_index == 0)
  {
    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494
    // joint-axis-reference-frame-doesnt-match
    Eigen::Isometry3d dtTransfJointLeftToParentLink
        = this->dtJoint->getTransformFromParentBodyNode().inverse();
    dtAxis = dtTransfJointLeftToParentLink.linear() * dtAxis;

    this->dtUniveralJoint->setAxis1(dtAxis);
  }
  else if (_index == 1)
  {
    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494
    // joint-axis-reference-frame-doesnt-match
    Eigen::Isometry3d dtTransfJointLeftToParentLink
        = this->dtJoint->getTransformFromParentBodyNode().inverse();
    dtAxis = dtTransfJointLeftToParentLink.linear() * dtAxis;

    this->dtUniveralJoint->setAxis2(dtAxis);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
math::Angle DARTUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (_index == 0)
  {
    double radianAngle = this->dtJoint->getGenCoord(0)->get_q();
    result.SetFromRadian(radianAngle);
  }
  else if (_index == 1)
  {
    double radianAngle = this->dtJoint->getGenCoord(1)->get_q();
    result.SetFromRadian(radianAngle);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (_index == 0)
    result = this->dtJoint->getGenCoord(0)->get_dq();
  else if (_index == 1)
    result = this->dtJoint->getGenCoord(1)->get_dq();
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (_index == 0)
    this->dtJoint->getGenCoord(0)->set_dq(_vel);
  else if (_index == 1)
    this->dtJoint->getGenCoord(1)->set_dq(_vel);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetMaxForce(unsigned int _index, double _force)
{
  if (_index == 0)
    this->dtJoint->getGenCoord(0)->set_tauMax(_force);
  else if (_index == 1)
    this->dtJoint->getGenCoord(1)->set_tauMax(_force);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetMaxForce(unsigned int _index)
{
  double result = 0.0;

  if (_index == 0)
    result = this->dtJoint->getGenCoord(0)->get_tauMax();
  else if (_index == 1)
    result = this->dtJoint->getGenCoord(1)->get_tauMax();
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dtJoint->getGenCoord(0)->set_tau(_effort);
  else if (_index == 1)
    this->dtJoint->getGenCoord(1)->set_tau(_effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}
