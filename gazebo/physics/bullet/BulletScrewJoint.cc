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
/* Desc: A bullet screw or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletScrewJoint::BulletScrewJoint(btDynamicsWorld *_world, BasePtr _parent)
    : ScrewJoint<BulletJoint>(_parent), bulletScrew(NULL)
{
  GZ_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
}

//////////////////////////////////////////////////
BulletScrewJoint::~BulletScrewJoint()
{
}

//////////////////////////////////////////////////
void BulletScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<BulletJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void BulletScrewJoint::Init()
{
  ScrewJoint<BulletJoint>::Init();

  gzwarn << "Screw joint constraints are currently not enforced" << "\n";

  BulletLinkPtr bulletChildLink =
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);


  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  math::Vector3 pivotA, pivotB;
  math::Pose pose;

  pivotA = this->anchorPos;
  pivotB = this->anchorPos;
  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotA -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotA = pose.rot.RotateVectorReverse(pivotA);
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotB -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotB = pose.rot.RotateVectorReverse(pivotB);
  }

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletScrew = new btSliderConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        frame1, frame2, true);
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletScrew = new btSliderConstraint(
        *bulletChildLink->GetBulletLink(), frame2, true);
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletScrew = new btSliderConstraint(
        *bulletParentLink->GetBulletLink(), frame1, true);
  }
  // Throw an error if no links are given.
  else
  {
    gzthrow("joint without links\n");
  }

  this->constraint = this->bulletScrew;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetVelocity(int /*_index*/) const
{
  double result = 0;
  if (this->bulletScrew)
    result = this->bulletScrew->getTargetLinMotorVelocity();
  return result;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetVelocity(int /*_index*/, double _angle)
{
  if (this->bulletScrew)
    this->bulletScrew->setTargetLinMotorVelocity(_angle);
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetForceImpl(int /*_index*/, double /*_force*/)
{
  gzlog << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetHighStop(int /*_index*/, const math::Angle &_angle)
{
  if (this->bulletScrew)
    this->bulletScrew->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetLowStop(int /*_index*/, const math::Angle &_angle)
{
  if (this->bulletScrew)
    this->bulletScrew->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getUpperLinLimit();
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getLowerLinLimit();
  return result;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetMaxForce(int /*_index*/, double _force)
{
  if (this->bulletScrew)
    this->bulletScrew->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetMaxForce(int /*index*/)
{
  double result = 0;
  if (this->bulletScrew)
    result = this->bulletScrew->getMaxLinMotorForce();
  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletScrewJoint::GetGlobalAxis(int /*_index*/) const
{
  math::Vector3 result;
  if (this->bulletScrew)
  {
    // I have not verified the following math, though I based it on internal
    // bullet code at line 250 of btHingeConstraint.cpp
    btVector3 vec =
      this->bulletScrew->getRigidBodyA().getCenterOfMassTransform().getBasis() *
      this->bulletScrew->getFrameOffsetA().getBasis().getColumn(2);
    result = BulletTypes::ConvertVector3(vec);
  }
  else
    gzwarn << "bulletHinge does not exist, returning fake axis\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getLinearPos();
  return result;
}
