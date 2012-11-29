/*
 * Copyright 2011 Nate Koenig
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
/* Desc: A simbody slider or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/simbody/simbody_inc.h"
#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodySliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodySliderJoint::SimbodySliderJoint(btDynamicsWorld *_world, BasePtr _parent)
    : SliderJoint<SimbodyJoint>(_parent)
{
  this->world = _world;
}

//////////////////////////////////////////////////
SimbodySliderJoint::~SimbodySliderJoint()
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodySliderJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  SliderJoint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  if (!simbodyChildLink || !simbodyParentLink)
    gzthrow("Requires simbody bodies");

  btVector3 anchor, axis1, axis2;
  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  math::Vector3 pivotA, pivotB;

  pivotA = this->anchorPos - this->parentLink->GetWorldPose().pos;
  pivotB = this->anchorPos - this->childLink->GetWorldPose().pos;

  pivotA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(pivotA);
  pivotB = this->childLink->GetWorldPose().rot.RotateVectorReverse(pivotB);

  std::cout << "AnchorPos[" << this->anchorPos << "]\n";
  std::cout << "Slider PivotA[" << pivotA << "] PivotB[" << pivotB << "]\n";

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  this->btSlider = new btSliderConstraint(
      *simbodyChildLink->GetSimbodyLink(),
      *simbodyParentLink->GetSimbodyLink(),
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
math::Angle SimbodySliderJoint::GetAngle(int /*_index*/) const
{
  return static_cast<btSliderConstraint*>(this->constraint)->getLinearPos();
}

//////////////////////////////////////////////////
double SimbodySliderJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetVelocity(int /*_index*/, double _angle)
{
  this->btSlider->setTargetLinMotorVelocity(_angle);
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetDamping(int /*index*/, const double _damping)
{
  this->btSlider->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetForce(int /*_index*/, double _force)
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
void SimbodySliderJoint::SetHighStop(int /*_index*/,
                                    const math::Angle &/*_angle*/)
{
  // this->btSlider->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetLowStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  // this->btSlider->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetHighStop(int /*_index*/)
{
  return this->btSlider->getUpperLinLimit();
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetLowStop(int /*_index*/)
{
  return this->btSlider->getLowerLinLimit();
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetMaxForce(int /*_index*/, double _force)
{
  this->btSlider->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double SimbodySliderJoint::GetMaxForce(int /*_index*/)
{
  return this->btSlider->getMaxLinMotorForce();
}

//////////////////////////////////////////////////
math::Vector3 SimbodySliderJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Angle();
}


