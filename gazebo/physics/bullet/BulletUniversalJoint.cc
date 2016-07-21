/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/bullet/BulletJointPrivate.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletUniversalJoint::BulletUniversalJoint(btDynamicsWorld *_world,
  BasePtr _parent) : UniversalJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld = _world;
  this->bulletUniversal = nullptr;
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
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->childLink);
  BulletLinkPtr bulletParentLink =
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->parentLink);

  ignition::math::Vector3d axis1 = this->initialWorldAxis[0];
  ignition::math::Vector3d axis2 = this->initialWorldAxis[1];

  // Check that axis1 and axis2 are orthogonal unit vectors
  if (ignition::math::equal(axis1.Length(), 0.0))
  {
    gzerr << "Joint [" << this->ScopedName()
          << "] axis1 must have non-zero length, aborting"
          << std::endl;
    return;
  }

  if (ignition::math::equal(axis2.Length(), 0.0))
  {
    gzerr << "Joint [" << this->ScopedName()
          << "] axis2 must have non-zero length, aborting"
          << std::endl;
    return;
  }

  if (ignition::math::equal(axis1.Cross(axis2).Length(), 0.0))
  {
    gzerr << "Joint [" << this->ScopedName()
          << "] axis1 and axis2 must not be parallel, aborting"
          << std::endl;
    return;
  }

  // Normalize axis unit vectors
  axis1.Normalize();
  axis2.Normalize();

  if (bulletChildLink && bulletParentLink)
  {
    this->bulletUniversal = new gzBtUniversalConstraint(
        *bulletParentLink->BtLink(),
        *bulletChildLink->BtLink(),
        btVector3(this->bulletJointDPtr->anchorPos.X(),
                  this->bulletJointDPtr->anchorPos.Y(),
                  this->bulletJointDPtr->anchorPos.Z()),
        btVector3(axis1.X(), axis1.Y(), axis1.Z()),
        btVector3(axis2.X(), axis2.Y(), axis2.Z()));
  }
  else if (bulletParentLink)
  {
    this->bulletUniversal = new gzBtUniversalConstraint(
        *bulletParentLink->BtLink(),
        btVector3(this->bulletJointDPtr->anchorPos.X(),
                  this->bulletJointDPtr->anchorPos.Y(),
                  this->bulletJointDPtr->anchorPos.Z()),
        btVector3(axis1.X(), axis1.Y(), axis1.Z()),
        btVector3(axis2.X(), axis2.Y(), axis2.Z()));
  }
  else if (bulletChildLink)
  {
    this->bulletUniversal = new gzBtUniversalConstraint(
        *bulletChildLink->BtLink(),
        btVector3(this->bulletJointDPtr->anchorPos.X(),
                  this->bulletJointDPtr->anchorPos.Y(),
                  this->bulletJointDPtr->anchorPos.Z()),
        btVector3(axis1.X(), axis1.Y(), axis1.Z()),
        btVector3(axis2.X(), axis2.Y(), axis2.Z()));
  }

  this->bulletJointDPtr->constraint = this->bulletUniversal;

  // Set angleOffset based on hinge angle at joint creation.
  // GetAngleImpl will report angles relative to this offset.
  this->angleOffset[0] = this->bulletUniversal->getAngle2();
  this->angleOffset[1] = this->bulletUniversal->getAngle1();

  // Get{Upp|Low}erLimit gets the original sdf values
  // Set{High|Low}Stop translates to bullet's axis definitions
  this->SetHighStop(0, this->UpperLimit(0));
  this->SetHighStop(1, this->UpperLimit(1));
  this->SetLowStop(0, this->LowerLimit(0));
  this->SetLowStop(1, this->LowerLimit(1));

  // Add the joint to the world
  GZ_ASSERT(this->bulletJointDPtr->bulletWorld, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld->addConstraint(
      this->bulletUniversal, true);

  // Allows access to impulse
  this->bulletUniversal->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletUniversalJoint::Anchor(
    const unsigned int /*index*/) const
{
  return this->bulletJointDPtr->anchorPos;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetAxis(const unsigned int _index,
                                   const ignition::math::Vector3d &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (!this->bulletUniversal)
  {
    if (_index < this->AngleCount())
    {
      // this hasn't been initialized yet, store axis in initialWorldAxis
      ignition::math::Quaterniond axisFrame = this->AxisFrame(_index);
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
double BulletUniversalJoint::Velocity(const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid joint axis index[" << _index << "], returning 0.\n";
    return 0;
  }

  double result = 0;
  ignition::math::Vector3d globalAxis = this->GlobalAxis(_index);

  if (this->bulletJointDPtr->childLink)
  {
    result += globalAxis.Dot(
        this->bulletJointDPtr->childLink->WorldAngularVel());
  }

  if (this->bulletJointDPtr->parentLink)
  {
    result -= globalAxis.Dot(
        this->bulletJointDPtr->parentLink->WorldAngularVel());
  }

  return result;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetVelocity(const unsigned int _index,
    const double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetForceImpl(const unsigned int _index,
    const double _effort)
{
  if (this->bulletUniversal)
  {
    int col;
    switch (_index)
    {
      case 0:
        col = 2;
        break;
      case 1:
        col = 1;
        break;
      default:
        gzerr << "Invalid axis index [" << _index << "].\n";
        return;
    }

    // z-axis of constraint frame
    btVector3 hingeAxisLocalA =
      this->bulletUniversal->getFrameOffsetA().getBasis().getColumn(col);

    btVector3 hingeAxisLocalB =
      this->bulletUniversal->getFrameOffsetB().getBasis().getColumn(col);

    btVector3 hingeAxisWorldA =
      this->bulletUniversal->getRigidBodyA().getWorldTransform().getBasis() *
      hingeAxisLocalA;

    btVector3 hingeAxisWorldB =
      this->bulletUniversal->getRigidBodyB().getWorldTransform().getBasis() *
      hingeAxisLocalB;

    btVector3 hingeTorqueA = _effort * hingeAxisWorldA;
    btVector3 hingeTorqueB = _effort * hingeAxisWorldB;

    this->bulletUniversal->getRigidBodyA().applyTorque(-hingeTorqueA);
    this->bulletUniversal->getRigidBodyB().applyTorque(hingeTorqueB);
  }
  else
    gzerr << "Trying to set force on a joint that has not been created\n";
}

//////////////////////////////////////////////////
bool BulletUniversalJoint::SetHighStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  // bullet does not handle joint angles near [-pi/2, +pi/2]
  // so artificially truncate it and let users know
  double angle = _angle.Radian();

  if (angle < -M_PI/2.1 || angle > M_PI/2.1)
  {
    angle = ignition::math::clamp(angle, -M_PI/2.1, M_PI/2.1);
    gzwarn << "Truncating joint limit [" << _angle.Radian()
           << "] to [" << angle << "] due to issue #1113.\n";
  }

  Joint::SetHighStop(_index, ignition::math::Angle(angle));
  if (this->bulletUniversal)
  {
    if (_index == 1)
    {
      this->bulletUniversal->setLowerLimit(
        this->angleOffset[0] - angle, -this->HighStop(0).Radian());
      return true;
    }
    else if (_index == 0)
    {
      this->bulletUniversal->setLowerLimit(
        -this->HighStop(1).Radian(), this->angleOffset[1] - angle);
      return true;
    }
    else
    {
      gzerr << "Invalid axis index [" << _index << "].\n";
      return false;
    }
  }
  else
  {
    gzerr << "bulletUniversal not yet created.\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool BulletUniversalJoint::SetLowStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  // bullet does not handle joint angles near [-pi/2, +pi/2]
  // so artificially truncate it and let users know
  double angle = _angle.Radian();

  if (angle < -M_PI/2.1 || angle > M_PI/2.1)
  {
    angle = ignition::math::clamp(angle, -M_PI/2.1, M_PI/2.1);
    gzwarn << "Truncating joint limit [" << _angle.Radian()
           << "] to [" << angle << "] due to issue #1113.\n";
  }

  Joint::SetLowStop(_index, ignition::math::Angle(angle));
  if (this->bulletUniversal)
  {
    if (_index == 1)
    {
      this->bulletUniversal->setUpperLimit(
        this->angleOffset[0] - angle, -this->LowStop(0).Radian());
      return true;
    }
    else if (_index == 0)
    {
      this->bulletUniversal->setUpperLimit(
        -this->LowStop(1).Radian(), this->angleOffset[1] - angle);
      return true;
    }
    else
    {
      gzerr << "Invalid axis index [" << _index << "].\n";
      return false;
    }
  }
  else
  {
    gzerr << "bulletUniversal not yet created.\n";
    return false;
  }
}

//////////////////////////////////////////////////
ignition::math::Angle BulletUniversalJoint::HighStop(
    const unsigned int _index) const
{
  ignition::math::Angle result;

  if (this->bulletUniversal)
  {
    btScalar limit1, limit2;
    this->bulletUniversal->getLowerLimit(limit1, limit2);
    if (_index == 1)
      result.Radian(-limit1);
    else if (_index == 0)
      result.Radian(-limit2);
    else
      gzerr << "Invalid axis index[" << _index << "]" << std::endl;
  }

  return result;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletUniversalJoint::LowStop(
    const unsigned int _index) const
{
  ignition::math::Angle result;

  if (this->bulletUniversal)
  {
    btScalar limit1, limit2;
    this->bulletUniversal->getUpperLimit(limit1, limit2);
    if (_index == 1)
      result.Radian(-limit1);
    else if (_index == 0)
      result.Radian(-limit2);
    else
      gzerr << "Invalid axis index[" << _index << "]" << std::endl;
  }

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletUniversalJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid joint axis index[" << _index << "]\n";
    return ignition::math::Vector3d::Zero;
  }

  ignition::math::Vector3d result = this->initialWorldAxis[_index];

  if (this->bulletUniversal)
  {
    if (_index == 0)
    {
      btVector3 vec = this->bulletUniversal->
        getRigidBodyA().getCenterOfMassTransform().getBasis() *
        this->bulletUniversal->getFrameOffsetA().getBasis().getColumn(2);

      result = BulletTypes::ConvertVector3Ign(vec);
    }
    else if (_index == 1)
    {
      btVector3 vec = this->bulletUniversal->
        getRigidBodyB().getCenterOfMassTransform().getBasis() *
        this->bulletUniversal->getFrameOffsetB().getBasis().getColumn(1);

      result = BulletTypes::ConvertVector3Ign(vec);
    }
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletUniversalJoint::AngleImpl(
    const unsigned int _index) const
{
  ignition::math::Angle result;

  if (this->bulletUniversal)
  {
    if (_index == 0)
      result = this->angleOffset[0] - this->bulletUniversal->getAngle2();
    else if (_index == 1)
      result = this->angleOffset[1] - this->bulletUniversal->getAngle1();
    else
      gzerr << "Invalid axis index[" << _index << "]\n";
  }
  else
    gzlog << "bulletUniversal does not yet exist" << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool BulletUniversalJoint::SetParam(const std::string &_key,
    const unsigned int _index,
    const boost::any &_value)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return false;
  }

  return BulletJoint::SetParam(_key, _index, _value);
}

//////////////////////////////////////////////////
double BulletUniversalJoint::Param(const std::string &_key,
                                   const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  return BulletJoint::Param(_key, _index);
}
