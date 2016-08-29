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
  math::Vector3 axis1 = axis1Elem->Get<math::Vector3>("xyz");

  sdf::ElementPtr axis2Elem = this->sdf->GetElement("axis2");
  math::Vector3 axis2 = axis2Elem->Get<math::Vector3>("xyz");

  // TODO: should check that axis1 and axis2 are orthogonal unit vectors

  btVector3 banchor(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z);
  btVector3 baxis1(axis1.x, axis1.y, axis1.z);
  btVector3 baxis2(axis2.x, axis2.y, axis2.z);

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
math::Vector3 BulletHinge2Joint::GetAnchor(unsigned int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
math::Vector3 BulletHinge2Joint::GetAxis(unsigned int /*index*/) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return math::Vector3();
  }

  btVector3 vec = this->bulletHinge2->getAxis1();
  return math::Vector3(vec.getX(), vec.getY(), vec.getZ());
}

//////////////////////////////////////////////////
math::Angle BulletHinge2Joint::GetAngle(unsigned int /*_index*/) const
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return math::Angle();
  }

  return this->bulletHinge2->getAngle1();
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
void BulletHinge2Joint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
{
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3 vec(_axis.x, _axis.y, _axis.z);
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
bool BulletHinge2Joint::SetHighStop(unsigned int /*_index*/,
    const math::Angle &_angle)
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
bool BulletHinge2Joint::SetLowStop(unsigned int /*_index*/,
    const math::Angle &_angle)
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
math::Angle BulletHinge2Joint::GetHighStop(unsigned int _index)
{
  if (!this->bulletHinge2)
  {
    gzerr << "Joint must be created first.\n";
    return math::Angle();
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
math::Angle BulletHinge2Joint::GetLowStop(unsigned int _index)
{
  if (!this->bulletHinge2)
  {
    gzerr << "BulletHinge2Joint::bulletHigne2 not created yet, returning 0.\n";
    return math::Angle(0.0);
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
math::Vector3 BulletHinge2Joint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle BulletHinge2Joint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "BulletHinge2Joint::GetAngleImpl not implemented\n";
  return math::Angle();
}
