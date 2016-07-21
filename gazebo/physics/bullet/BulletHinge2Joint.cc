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
#include <sdf/sdf.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletJointPrivate.hh"

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletHinge2Joint::BulletHinge2Joint(btDynamicsWorld *_world, BasePtr _parent)
: Hinge2Joint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld = _world;
  this->bulletHinge2 = nullptr;
  this->angleOffset[0] = 0.0;
  this->angleOffset[1] = 0.0;
}

//////////////////////////////////////////////////
BulletHinge2Joint::~BulletHinge2Joint()
{
}

//////////////////////////////////////////////////
void BulletHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletHinge2Joint::Init()
{
  Hinge2Joint<BulletJoint>::Init();
  BulletLinkPtr bulletChildLink =
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->childLink);
  BulletLinkPtr bulletParentLink =
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->parentLink);

  if (!bulletParentLink)
  {
    gzerr << "BulletHinge2Joint cannot be connected to the world (parent)\n";
    return;
  }

  if (!bulletChildLink)
  {
    gzerr << "BulletHinge2Joint cannot be connected to the world (child)\n";
    return;
  }

  sdf::ElementPtr axis1Elem = this->bulletJointDPtr->sdf->GetElement("axis");
  ignition::math::Vector3d axis1 =
    axis1Elem->Get<ignition::math::Vector3d>("xyz");

  sdf::ElementPtr axis2Elem = this->bulletJointDPtr->sdf->GetElement("axis2");
  ignition::math::Vector3d axis2 =
    axis2Elem->Get<ignition::math::Vector3d>("xyz");

  // TODO: should check that axis1 and axis2 are orthogonal unit vectors

  btVector3 banchor(this->bulletJointDPtr->anchorPos.X(),
      this->bulletJointDPtr->anchorPos.Y(),
      this->bulletJointDPtr->anchorPos.Z());

  btVector3 baxis1(axis1.X(), axis1.Y(), axis1.Z());
  btVector3 baxis2(axis2.X(), axis2.Y(), axis2.Z());

  this->bulletHinge2 = new btHinge2Constraint(
      *bulletParentLink->BtLink(), *bulletChildLink->BtLink(),
      banchor, baxis1, baxis2);

  this->bulletJointDPtr->constraint = this->bulletHinge2;

  // Add the joint to the world
  GZ_ASSERT(this->bulletJointDPtr->bulletWorld,
      "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld->addConstraint(
      this->bulletJointDPtr->constraint, true);

  // Allows access to impulse
  this->bulletJointDPtr->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHinge2Joint::Anchor(
    const unsigned int /*index*/) const
{
  return this->bulletJointDPtr->anchorPos;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHinge2Joint::Axis(
    const unsigned int /*index*/) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return ignition::math::Vector3d();
  }

  btVector3 vec = this->bulletHinge2->getAxis1();
  return ignition::math::Vector3d(vec.getX(), vec.getY(), vec.getZ());
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::Angle(
    const unsigned int /*_index*/) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return ignition::math::Angle::Zero;
  }

  return this->bulletHinge2->getAngle1();
}

//////////////////////////////////////////////////
double BulletHinge2Joint::Velocity(const unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::Velocity not implemented" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetVelocity(const unsigned int /*_index*/,
    const double /*_angle*/)
{
  gzerr << "BulletHinge2Joint::SetVelocity not implemented" << std::endl;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3 vec(_axis.x, _axis.y, _axis.z);
  ((btHingeConstraint*)this->btHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetForceImpl(const unsigned int /*_index*/,
    const double /*_torque*/)
{
  gzerr << "BulletHinge2Joint::SetForceImpl not implemented" << std::endl;
}

//////////////////////////////////////////////////
bool BulletHinge2Joint::SetHighStop(const unsigned int /*_index*/,
    const ignition::math::Angle &_angle)
{
  if (this->bulletHinge2)
  {
    this->bulletHinge2->setUpperLimit(_angle.Radian());
    return true;
  }
  else
  {
    gzerr << "Joint must be created first.\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool BulletHinge2Joint::SetLowStop(const unsigned int /*_index*/,
    const ignition::math::Angle &_angle)
{
  if (this->bulletHinge2)
  {
    this->bulletHinge2->setLowerLimit(_angle.Radian());
    return true;
  }
  else
  {
    gzerr << "Joint must be created first.\n";
    return false;
  }
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::HighStop(
    const unsigned int _index) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return ignition::math::Angle::Zero;
  }

#ifndef LIBBULLET_VERSION_GT_282
  btRotationalLimitMotor *motor;
#else
  btRotationalLimitMotor2 *motor;
#endif

  motor = this->bulletHinge2->getRotationalLimitMotor(_index);

  if (motor)
    return motor->m_hiLimit;

  gzerr << "Unable to get high stop for axis _index[" << _index << "]\n";
  return 0;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::LowStop(
    const unsigned int _index) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "BulletHinge2Joint::bulletHigne2 not created yet, returning 0.\n";
    return ignition::math::Angle::Zero;
  }

#ifndef LIBBULLET_VERSION_GT_282
  btRotationalLimitMotor *motor;
#else
  btRotationalLimitMotor2 *motor;
#endif

  motor = this->bulletHinge2->getRotationalLimitMotor(_index);

  if (motor)
    return motor->m_loLimit;

  gzerr << "Unable to get high stop for axis _index[" << _index << "]\n";
  return 0;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHinge2Joint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::GetGlobalAxis not implemented\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::AngleImpl(
    const unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::GetAngleImpl not implemented\n";
  return ignition::math::Angle::Zero;
}
