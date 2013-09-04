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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletBallJoint::BulletBallJoint(btDynamicsWorld *_world, BasePtr _parent)
    : BallJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
  this->bulletBall = NULL;
}

//////////////////////////////////////////////////
BulletBallJoint::~BulletBallJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 BulletBallJoint::GetAnchor(int /*_index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
void BulletBallJoint::SetAnchor(int /*_index*/,
                                const math::Vector3 &/*_anchor*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletBallJoint::Init()
{
  BallJoint<BulletJoint>::Init();

  BulletLinkPtr bulletChildLink =
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  if (!bulletChildLink || !bulletParentLink)
    gzthrow("Requires bullet bodies");

  math::Vector3 pivotA, pivotB;

  // Compute the pivot point, based on the anchorPos
  pivotA = this->anchorPos + this->childLink->GetWorldPose().pos
                           - this->parentLink->GetWorldPose().pos;
  pivotB = this->anchorPos;

  this->bulletBall = new btPoint2PointConstraint(
      *bulletParentLink->GetBulletLink(),
      *bulletChildLink->GetBulletLink(),
      btVector3(pivotA.x, pivotA.y, pivotA.z),
      btVector3(pivotB.x, pivotB.y, pivotB.z));

  this->constraint = this->bulletBall;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

/////////////////////////////////////////////////
void BulletBallJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double BulletBallJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
double BulletBallJoint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
void BulletBallJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented\n";
  return;
}

/////////////////////////////////////////////////
math::Vector3 BulletBallJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Vector3();
}

/////////////////////////////////////////////////
math::Angle BulletBallJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void BulletBallJoint::SetForceImpl(int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletBallJoint::SetHighStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  if (this->bulletBall)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->bulletBall->setLimit(this->btBall->getLowerLimit(),
    //                         _angle.Radian());
  }
  else
  {
    gzthrow("Joint must be created first");
  }
}

//////////////////////////////////////////////////
void BulletBallJoint::SetLowStop(int /*_index*/,
                                  const math::Angle &/*_angle*/)
{
  if (this->bulletBall)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->bulletBall->setLimit(-_angle.Radian(),
    //                         this->bulletBall->getUpperLimit());
  }
  else
    gzthrow("Joint must be created first");
}

