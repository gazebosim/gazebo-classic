/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletUniversalJoint::BulletUniversalJoint(btDynamicsWorld *_world,
  BasePtr _parent) : UniversalJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
  this->bulletUniversal = NULL;
}

//////////////////////////////////////////////////
BulletUniversalJoint::~BulletUniversalJoint()
{
}

//////////////////////////////////////////////////
void BulletUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletUniversalJoint::Init()
{
  UniversalJoint<BulletJoint>::Init();

  BulletLinkPtr bulletChildLink =
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  math::Vector3 axis1 = this->initialWorldAxis[0];
  math::Vector3 axis2 = this->initialWorldAxis[1];

  // TODO: should check that axis1 and axis2 are orthogonal unit vectors

  if (bulletChildLink && bulletParentLink)
  {
    this->bulletUniversal = new gzBtUniversalConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        btVector3(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z),
        btVector3(axis1.x, axis1.y, axis1.z),
        btVector3(axis2.x, axis2.y, axis2.z));
  }
  else if (bulletParentLink)
  {
    this->bulletUniversal = new gzBtUniversalConstraint(
        *bulletParentLink->GetBulletLink(),
        btVector3(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z),
        btVector3(axis1.x, axis1.y, axis1.z),
        btVector3(axis2.x, axis2.y, axis2.z));
  }
  else if (bulletChildLink)
  {
    this->bulletUniversal = new gzBtUniversalConstraint(
        *bulletChildLink->GetBulletLink(),
        btVector3(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z),
        btVector3(axis1.x, axis1.y, axis1.z),
        btVector3(axis2.x, axis2.y, axis2.z));
  }

  this->constraint = this->bulletUniversal;

  // Set angleOffset based on hinge angle at joint creation.
  // GetAngleImpl will report angles relative to this offset.
  this->angleOffset[0] = this->bulletUniversal->getAngle1();
  this->angleOffset[1] = this->bulletUniversal->getAngle2();

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->bulletUniversal, true);

  // Allows access to impulse
  this->bulletUniversal->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
math::Vector3 BulletUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetAxis(unsigned int _index,
                                   const math::Vector3 &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (!this->bulletUniversal)
  {
    if (_index < 2)
    {
      // this hasn't been initialized yet, store axis in initialWorldAxis
      math::Quaternion axisFrame = this->GetAxisFrame(_index);
      this->initialWorldAxis[_index] = axisFrame.RotateVector(_axis);
    }
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }
}

//////////////////////////////////////////////////
double BulletUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;
  math::Vector3 globalAxis = this->GetGlobalAxis(_index);
  if (this->childLink)
    result += globalAxis.Dot(this->childLink->GetWorldAngularVel());
  if (this->parentLink)
    result -= globalAxis.Dot(this->parentLink->GetWorldAngularVel());
  return result;

  return 0;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetVelocity(unsigned int /*_index*/,
    double /*_angle*/)
{
  // gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetForceImpl(unsigned int /*_index*/,
    double /*_torque*/)
{
  // gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetMaxForce(unsigned int _index, double _t)
{
  if (this->bulletUniversal)
  {
    if (_index == 0)
      this->bulletUniversal->setMaxMotorImpulse1(_t);
    else if (_index == 1)
      this->bulletUniversal->setMaxMotorImpulse2(_t);
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }
  else
    gzerr << "bulletUniversal does not yet exist" << std::endl;
}

//////////////////////////////////////////////////
double BulletUniversalJoint::GetMaxForce(unsigned int _index)
{
  double result = 0;
  if (this->bulletUniversal)
  {
    if (_index == 0)
      result = this->bulletUniversal->getMaxMotorImpulse1();
    else if (_index == 1)
      result = this->bulletUniversal->getMaxMotorImpulse2();
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }
  else
    gzerr << "bulletUniversal does not yet exist" << std::endl;

  return result;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetHighStop(unsigned int _index,
    const math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);
  if (this->bulletUniversal)
  {
    if (_index == 0)
      this->bulletUniversal->setUpperLimit(
        _angle.Radian(), this->GetHighStop(1).Radian());
    else
      this->bulletUniversal->setUpperLimit(
        this->GetHighStop(0).Radian(), _angle.Radian());
  }
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetLowStop(unsigned int _index,
    const math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);
  if (this->bulletUniversal)
  {
    if (_index == 0)
      this->bulletUniversal->setLowerLimit(
        _angle.Radian(), this->GetLowStop(1).Radian());
    else
      this->bulletUniversal->setUpperLimit(
        this->GetLowStop(0).Radian(), _angle.Radian());
  }
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetHighStop(unsigned int _index)
{
  math::Angle result;

  if (this->bulletUniversal)
  {
    btRotationalLimitMotor *motor;
    motor = this->bulletUniversal->getRotationalLimitMotor(_index);
    result = motor->m_hiLimit;
  }
  else
    gzerr << "Joint must be created first\n";

  return result;
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetLowStop(unsigned int _index)
{
  math::Angle result;

  if (this->bulletUniversal)
  {
    btRotationalLimitMotor *motor;
    motor = this->bulletUniversal->getRotationalLimitMotor(_index);
    result = motor->m_loLimit;
  }
  else
    gzerr << "Joint must be created first\n";

  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  if (_index >= 2)
  {
    gzerr << "Invalid joint axis index[" << _index << "]\n";
    return math::Vector3::Zero;
  }

  math::Vector3 result = this->initialWorldAxis[_index];

  if (this->bulletUniversal)
  {
    if (_index == 0)
    {
      btVector3 vec = this->bulletUniversal->
        getRigidBodyA().getCenterOfMassTransform().getBasis() *
        this->bulletUniversal->getFrameOffsetA().getBasis().getColumn(2);

      result = BulletTypes::ConvertVector3(vec);
    }
    else if (_index == 1)
    {
      btVector3 vec = this->bulletUniversal->
        getRigidBodyB().getCenterOfMassTransform().getBasis() *
        this->bulletUniversal->getFrameOffsetB().getBasis().getColumn(1);

      result = BulletTypes::ConvertVector3(vec);
    }
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (this->bulletUniversal)
  {
    if (_index == 0)
      result = this->angleOffset[0] - this->bulletUniversal->getAngle1();
    else if (_index == 1)
      result = this->angleOffset[1] - this->bulletUniversal->getAngle2();
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }
  else
    gzerr << "bulletUniversal does not yet exist" << std::endl;

  return result;
}
