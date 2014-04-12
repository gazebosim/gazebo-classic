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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "ignition/common/Assert.hh"
#include "ignition/common/Console.hh"
#include "ignition/common/Exception.hh"

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
  IGN_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
  this->bulletHinge2 = NULL;
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
    ignthrow("BulletHinge2Joint cannot be connected to the world (parent)");
  if (!bulletChildLink)
    ignthrow("BulletHinge2Joint cannot be connected to the world (child)");

  sdf::ElementPtr axis1Elem = this->sdf->GetElement("axis");
  ignition::math::Vector3 axis1 =
    axis1Elem->Get<ignition::math::Vector3>("xyz");

  sdf::ElementPtr axis2Elem = this->sdf->GetElement("axis2");
  ignition::math::Vector3 axis2 =
    axis2Elem->Get<ignition::math::Vector3>("xyz");

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
  IGN_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->constraint, true);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3 BulletHinge2Joint::GetAnchor(
    unsigned int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
ignition::math::Vector3 BulletHinge2Joint::GetAxis(unsigned int /*index*/) const
{
  if (!this->bulletHinge2)
  {
    ignerr << "Joint must be created first.\n";
    return ignition::math::Vector3();
  }

  btVector3 vec = this->bulletHinge2->getAxis1();
  return ignition::math::Vector3(vec.getX(), vec.getY(), vec.getZ());
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::GetAngle(unsigned int /*_index*/) const
{
  if (!this->bulletHinge2)
  {
    ignerr << "Joint must be created first.\n";
    return ignition::math::Angle();
  }

  return this->bulletHinge2->getAngle1();
}

//////////////////////////////////////////////////
double BulletHinge2Joint::GetVelocity(unsigned int /*_index*/) const
{
  ignerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  ignerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetAxis(unsigned int /*_index*/,
    const ignition::math::Vector3 &/*_axis*/)
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
  ignerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  ignerr << "Not implemented";
}

//////////////////////////////////////////////////
double BulletHinge2Joint::GetMaxForce(unsigned int /*_index*/)
{
  ignerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetHighStop(unsigned int /*_index*/,
    const ignition::math::Angle &_angle)
{
  if (this->bulletHinge2)
    this->bulletHinge2->setUpperLimit(_angle.Radian());
  else
    ignerr << "Joint must be created first.\n";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetLowStop(unsigned int /*_index*/,
    const ignition::math::Angle &_angle)
{
  if (this->bulletHinge2)
    this->bulletHinge2->setLowerLimit(_angle.Radian());
  else
    ignerr << "Joint must be created first.\n";
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::GetHighStop(unsigned int _index)
{
  if (!this->bulletHinge2)
  {
    ignerr << "Joint must be created first.\n";
    return ignition::math::Angle();
  }

  btRotationalLimitMotor *motor =
    this->bulletHinge2->getRotationalLimitMotor(_index);
  if (motor)
    return motor->m_hiLimit;

  ignerr << "Unable to get high stop for axis _index[" << _index << "]\n";
  return 0;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::GetLowStop(unsigned int _index)
{
  if (!this->bulletHinge2)
    ignerr << "Joint must be created first.\n";

  btRotationalLimitMotor *motor =
    this->bulletHinge2->getRotationalLimitMotor(_index);
  if (motor)
    return motor->m_loLimit;

  ignerr << "Unable to get high stop for axis _index[" << _index << "]\n";
  return 0;
}

//////////////////////////////////////////////////
ignition::math::Vector3 BulletHinge2Joint::GetGlobalAxis(
    unsigned int /*_index*/) const
{
  ignerr << "BulletHinge2Joint::GetGlobalAxis not implemented\n";
  return ignition::math::Vector3();
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHinge2Joint::GetAngleImpl(
    unsigned int /*_index*/) const
{
  ignerr << "BulletHinge2Joint::GetAngleImpl not implemented\n";
  return ignition::math::Angle();
}
