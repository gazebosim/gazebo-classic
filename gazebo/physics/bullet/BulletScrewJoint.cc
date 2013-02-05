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
/* Desc: A bullet screw or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletTypes.hh"
#include "physics/bullet/BulletScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletScrewJoint::BulletScrewJoint(btDynamicsWorld *_world, BasePtr _parent)
    : ScrewJoint<BulletJoint>(_parent)
{
  this->world = _world;
  this->bulletScrew = NULL;
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
void BulletScrewJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  if (this->constraint)
    this->Detach();

  ScrewJoint<BulletJoint>::Attach(_one, _two);

  BulletLinkPtr bulletChildLink =
    boost::shared_static_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::shared_static_cast<BulletLink>(this->parentLink);

  if (!bulletChildLink || !bulletParentLink)
    gzthrow("Requires bullet bodies");

  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  math::Vector3 pivotA, pivotB;

  pivotA = this->anchorPos + this->childLink->GetWorldPose().pos
                           - this->parentLink->GetWorldPose().pos;
  pivotB = this->anchorPos;

  pivotA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(pivotA);
  pivotB = this->childLink->GetWorldPose().rot.RotateVectorReverse(pivotB);

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  this->bulletScrew = new btSliderConstraint(
      *bulletChildLink->GetBulletLink(),
      *bulletParentLink->GetBulletLink(),
      frame2, frame1, true);

  this->constraint = this->bulletScrew;

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetHighStop(int /*_index*/, const math::Angle &_angle)
{
  this->bulletScrew->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetLowStop(int /*_index*/, const math::Angle &_angle)
{
  this->bulletScrew->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetHighStop(int /*_index*/)
{
  return this->bulletScrew->getUpperLinLimit();
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetLowStop(int /*_index*/)
{
  return this->bulletScrew->getLowerLinLimit();
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetMaxForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetMaxForce(int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 BulletScrewJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "BulletScrewJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getLinearPos();
  return result;
}
