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
/* Desc: A universal joint
 * Author: Nate Koenig
 * Date: 24 May 2009
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

  if (!bulletParentLink)
    gzthrow("BulletUniversalJoint cannot be connected to the world (parent)");
  if (!bulletChildLink)
    gzthrow("BulletUniversalJoint cannot be connected to the world (child)");

  sdf::ElementPtr axis1Elem = this->sdf->GetElement("axis");
  math::Vector3 axis1 = axis1Elem->Get<math::Vector3>("xyz");

  sdf::ElementPtr axis2Elem = this->sdf->GetElement("axis2");
  math::Vector3 axis2 = axis2Elem->Get<math::Vector3>("xyz");

  // TODO: should check that axis1 and axis2 are orthogonal unit vectors

  this->bulletUniversal = new btUniversalConstraint(
      *bulletParentLink->GetBulletLink(),
      *bulletChildLink->GetBulletLink(),
      btVector3(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z),
      btVector3(axis1.x, axis1.y, axis1.z),
      btVector3(axis2.x, axis2.y, axis2.z));

  this->constraint = this->bulletUniversal;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->bulletUniversal, true);

  // Allows access to impulse
  this->bulletUniversal->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
math::Vector3 BulletUniversalJoint::GetAnchor(int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
math::Vector3 BulletUniversalJoint::GetAxis(int _index) const
{
  btVector3 axis = this->bulletUniversal->getAxis(_index);
  return math::Vector3(axis.getX(), axis.getY(), axis.getZ());
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetAxis(int /*_index*/,
                                   const math::Vector3 &/*_axis*/)
{
  // The anchor (pivot in Bullet lingo), can only be set on creation
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetAngle(int _index) const
{
  if (_index == 0)
    return this->bulletUniversal->getAngle1();
  else
    return this->bulletUniversal->getAngle2();
}

//////////////////////////////////////////////////
double BulletUniversalJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetForceImpl(int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletUniversalJoint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetHighStop(int _index, const math::Angle &_angle)
{
  if (this->bulletUniversal)
  {
    if (_index == 0)
      this->bulletUniversal->setUpperLimit(
        _angle.Radian(), this->GetHighStop(1).Radian());
    else
      this->bulletUniversal->setUpperLimit(
        this->GetHighStop(0).Radian(), _angle.Radian());
  }
  else
    gzerr << "bulletUniversal does not yet exist" << std::endl;
}

//////////////////////////////////////////////////
void BulletUniversalJoint::SetLowStop(int _index, const math::Angle &_angle)
{
  if (this->bulletUniversal)
  {
    if (_index == 0)
      this->bulletUniversal->setLowerLimit(
        _angle.Radian(), this->GetLowStop(1).Radian());
    else
      this->bulletUniversal->setUpperLimit(
        this->GetLowStop(0).Radian(), _angle.Radian());
  }
  else
    gzerr << "bulletUniversal does not yet exist" << std::endl;
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetHighStop(int _index)
{
  math::Angle result;

  if (this->bulletUniversal)
  {
    btRotationalLimitMotor *motor;
    motor = this->bulletUniversal->getRotationalLimitMotor(_index);
    result = motor->m_hiLimit;
  }
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetLowStop(int _index)
{
  math::Angle result;

  if (this->bulletUniversal)
  {
    btRotationalLimitMotor *motor;
    motor = this->bulletUniversal->getRotationalLimitMotor(_index);
    result = motor->m_loLimit;
  }
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletUniversalJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "BulletUniversalJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle BulletUniversalJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "BulletUniversalJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
