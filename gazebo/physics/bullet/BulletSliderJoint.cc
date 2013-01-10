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
/* Desc: A bullet slider or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletSliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletSliderJoint::BulletSliderJoint(btDynamicsWorld *_world, BasePtr _parent)
    : SliderJoint<BulletJoint>(_parent)
{
  this->world = _world;
  this->btSlider = NULL;
}

//////////////////////////////////////////////////
BulletSliderJoint::~BulletSliderJoint()
{
}

//////////////////////////////////////////////////
void BulletSliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletSliderJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  SliderJoint<BulletJoint>::Attach(_one, _two);

  BulletLinkPtr bulletChildLink =
    boost::shared_static_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::shared_static_cast<BulletLink>(this->parentLink);

  if (!bulletChildLink || !bulletParentLink)
    gzthrow("Requires bullet bodies");

  btVector3 anchor, axis1, axis2;
  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  math::Vector3 pivotA, pivotB;

  pivotA = this->anchorPos + this->childLink->GetWorldPose().pos
                           - this->parentLink->GetWorldPose().pos;
  pivotB = this->anchorPos;

  pivotA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(pivotA);
  pivotB = this->childLink->GetWorldPose().rot.RotateVectorReverse(pivotB);

  std::cout << "AnchorPos[" << this->anchorPos << "]\n";
  std::cout << "Slider PivotA[" << pivotA << "] PivotB[" << pivotB << "]\n";

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  this->btSlider = new btSliderConstraint(
      *bulletChildLink->GetBulletLink(),
      *bulletParentLink->GetBulletLink(),
      frame2, frame1, true);

  // this->btSlider->setLowerAngLimit(0.0);
  // this->btSlider->setUpperAngLimit(0.0);

  double pos = this->btSlider->getLinearPos();
  this->btSlider->setLowerLinLimit(pos);
  this->btSlider->setUpperLinLimit(pos+0.9);

  this->constraint = this->btSlider;

  // Add the joint to the world
  this->world->addConstraint(this->btSlider, true);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetAngle(int /*_index*/) const
{
  return static_cast<btSliderConstraint*>(this->constraint)->getLinearPos();
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetVelocity(int /*_index*/, double _angle)
{
  this->btSlider->setTargetLinMotorVelocity(_angle);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetDamping(int /*index*/, const double _damping)
{
  this->btSlider->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetForce(int /*_index*/, double _force)
{
  /*btVector3 hingeAxisLocal = this->btSlider->getAFrame().getBasis().getColumn(2); // z-axis of constraint frame
  btVector3 hingeAxisWorld = this->btSlider->getRigidBodyA().getWorldTransform().getBasis() * hingeAxisLocal;

  btVector3 hingeTorque = _torque * hingeAxisWorld;
  */

  btVector3 force(0, 0, _force);
  this->constraint->getRigidBodyA().applyCentralForce(force);
  this->constraint->getRigidBodyB().applyCentralForce(-force);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetHighStop(int /*_index*/,
                                    const math::Angle &/*_angle*/)
{
  // this->btSlider->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetLowStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  // this->btSlider->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetHighStop(int /*_index*/)
{
  return this->btSlider->getUpperLinLimit();
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetLowStop(int /*_index*/)
{
  return this->btSlider->getLowerLinLimit();
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetMaxForce(int /*_index*/, double _force)
{
  this->btSlider->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetMaxForce(int /*_index*/)
{
  return this->btSlider->getMaxLinMotorForce();
}

//////////////////////////////////////////////////
math::Vector3 BulletSliderJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Angle();
}


