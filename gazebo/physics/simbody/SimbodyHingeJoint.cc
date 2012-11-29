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
/* Desc: A SimbodyHingeJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */
#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodyHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHingeJoint::SimbodyHingeJoint(btDynamicsWorld *_world, BasePtr _parent)
    : HingeJoint<SimbodyJoint>(_parent)
{
  this->world = _world;
}

//////////////////////////////////////////////////
SimbodyHingeJoint::~SimbodyHingeJoint()
{
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  HingeJoint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  if (!simbodyChildLink || !simbodyParentLink)
    gzthrow("Requires simbody bodies");

  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  math::Vector3 axis = axisElem->GetValueVector3("xyz");

  math::Vector3 pivotA, pivotB, axisA, axisB;

  // Compute the pivot point, based on the anchorPos
  pivotA = this->anchorPos - this->parentLink->GetWorldPose().pos;
  pivotB = this->anchorPos - this->childLink->GetWorldPose().pos;

  pivotA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(pivotA);
  pivotB = this->childLink->GetWorldPose().rot.RotateVectorReverse(pivotB);

  axisA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(axis);
  axisA = axisA.Round();

  axisB = this->childLink->GetWorldPose().rot.RotateVectorReverse(axis);
  axisB = axisB.Round();

  this->btHinge = new btHingeConstraint(
      *simbodyParentLink->GetSimbodyLink(),
      *simbodyChildLink->GetSimbodyLink(),
      btVector3(pivotA.x, pivotA.y, pivotA.z),
      btVector3(pivotB.x, pivotB.y, pivotB.z),
      btVector3(axisA.x, axisA.y, axisA.z),
      btVector3(axisB.x, axisB.y, axisB.z));

  this->constraint = this->btHinge;

  double angle = this->btHinge->getHingeAngle();
  this->btHinge->setLimit(angle - .4, angle + .4);
  // Add the joint to the world
  this->world->addConstraint(this->btHinge, true);

  // Allows access to impulse
  this->btHinge->enableFeedback(true);
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHingeJoint::GetAnchor(int /*_index*/) const
{
  btTransform trans = this->btHinge->getAFrame();
  trans.getOrigin() +=
    this->btHinge->getRigidBodyA().getCenterOfMassTransform().getOrigin();
  return math::Vector3(trans.getOrigin().getX(),
      trans.getOrigin().getY(), trans.getOrigin().getZ());
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAnchor(int /*_index*/,
                                 const math::Vector3 &/*_anchor*/)
{
  // The anchor (pivot in Simbody lingo), can only be set on creation
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3 vec(_axis.x, _axis.y, _axis.z);
  ((btHingeConstraint*)this->btHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetAngle(int /*_index*/) const
{
  if (this->btHinge)
    return this->btHinge->getHingeAngle();
  else
    gzthrow("Joint has not been created");
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  // this->btHinge->enableAngularMotor(true, -_angle,
  // this->GetMaxForce(_index));
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetMaxForce(int /*_index*/, double _t)
{
  this->btHinge->setMaxMotorImpulse(_t);
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetMaxForce(int /*_index*/)
{
  return this->btHinge->getMaxMotorImpulse();
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetForce(int /*_index*/, double _torque)
{
  // math::Vector3 axis = this->GetLocalAxis(_index);
  // this->btHinge->enableAngularMotor(true);

  // z-axis of constraint frame
  btVector3 hingeAxisLocal =
    this->btHinge->getAFrame().getBasis().getColumn(2);

  btVector3 hingeAxisWorld =
    this->btHinge->getRigidBodyA().getWorldTransform().getBasis() *
    hingeAxisLocal;

  btVector3 hingeTorque = _torque * hingeAxisWorld;

  this->btHinge->getRigidBodyA().applyTorque(hingeTorque);
  this->btHinge->getRigidBodyB().applyTorque(-hingeTorque);
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetForce(int /*_index*/)
{
  return this->btHinge->getAppliedImpulse();
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetHighStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  if (this->btHinge)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->btHinge->setLimit(this->btHinge->getLowerLimit(),
    //                         _angle.Radian());
  }
  else
  {
    gzthrow("Joint must be created first");
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetLowStop(int /*_index*/,
                                  const math::Angle &/*_angle*/)
{
  if (this->btHinge)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->btHinge->setLimit(-_angle.Radian(),
    //                         this->btHinge->getUpperLimit());
  }
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;

  if (this->btHinge)
    result = this->btHinge->getUpperLimit();
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  if (this->btHinge)
    result = this->btHinge->getLowerLimit();
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHingeJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "SimbodyHingeJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "SimbodyHingeJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
