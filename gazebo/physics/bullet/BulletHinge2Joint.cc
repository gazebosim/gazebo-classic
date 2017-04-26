/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

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
  this->bulletWorld = _world;
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
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  if (!bulletParentLink)
    gzthrow("BulletHinge2Joint cannot be connected to the world (parent)");
  if (!bulletChildLink)
    gzthrow("BulletHinge2Joint cannot be connected to the world (child)");

  sdf::ElementPtr axis1Elem = this->sdf->GetElement("axis");
  auto axis1 = axis1Elem->Get<ignition::math::Vector3d>("xyz");

  sdf::ElementPtr axis2Elem = this->sdf->GetElement("axis2");
  auto axis2 = axis2Elem->Get<ignition::math::Vector3d>("xyz");

  // TODO: should check that axis1 and axis2 are orthogonal unit vectors

  btVector3 banchor(this->anchorPos.X(), this->anchorPos.Y(),
                    this->anchorPos.Z());
  btVector3 baxis1(axis1.X(), axis1.Y(), axis1.Z());
  btVector3 baxis2(axis2.X(), axis2.Y(), axis2.Z());

  this->bulletHinge2 = new btHinge2Constraint(
      *bulletParentLink->GetBulletLink(),
      *bulletChildLink->GetBulletLink(),
      banchor, baxis1, baxis2);

  this->constraint = this->bulletHinge2;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is null");
  this->bulletWorld->addConstraint(this->constraint, true);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHinge2Joint::Anchor(
    const unsigned int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
double BulletHinge2Joint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::GetVelocity not implemented" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "BulletHinge2Joint::SetVelocity not implemented" << std::endl;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3d vec(_axis.X(), _axis.Y(), _axis.Z());
  ((btHingeConstraint*)this->btHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetForceImpl(unsigned int /*_index*/,
    double /*_torque*/)
{
  gzerr << "BulletHinge2Joint::SetForceImpl not implemented" << std::endl;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetUpperLimit(const unsigned int /*_index*/,
    const double _limit)
{
  /// \todo Shouldn't index be taken into account for a hinge2 joint?
  if (this->bulletHinge2)
  {
    this->bulletHinge2->setUpperLimit(_limit);
  }
  else
  {
    gzerr << "Joint must be created first.\n";
  }
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetLowerLimit(const unsigned int /*_index*/,
    const double _limit)
{
  /// \todo Shouldn't index be taken into account for a hinge2 joint?
  if (this->bulletHinge2)
  {
    this->bulletHinge2->setLowerLimit(_limit);
  }
  else
  {
    gzerr << "Joint must be created first.\n";
  }
}

//////////////////////////////////////////////////
double BulletHinge2Joint::UpperLimit(const unsigned int _index) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return ignition::math::NAN_D;
  }

#ifndef LIBBULLET_VERSION_GT_282
  btRotationalLimitMotor *motor;
#else
  btRotationalLimitMotor2 *motor;
#endif
  motor = this->bulletHinge2->getRotationalLimitMotor(_index);
  if (motor)
    return motor->m_hiLimit;

  gzerr << "Unable to get upper limit for axis _index[" << _index << "]\n";
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double BulletHinge2Joint::LowerLimit(const unsigned int _index) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return ignition::math::NAN_D;
  }

#ifndef LIBBULLET_VERSION_GT_282
  btRotationalLimitMotor *motor;
#else
  btRotationalLimitMotor2 *motor;
#endif
  motor = this->bulletHinge2->getRotationalLimitMotor(_index);
  if (motor)
    return motor->m_loLimit;

  gzerr << "Unable to get lower limit for axis _index[" << _index << "]\n";
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHinge2Joint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::GlobalAxis not implemented\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
double BulletHinge2Joint::PositionImpl(const unsigned int /*_index*/) const
{
  /// \todo Copied from old BulletHinge2Joint::GetAngle, but it probably should
  /// return the value according to the index
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return ignition::math::NAN_D;
  }

  return this->bulletHinge2->getAngle1();
}
