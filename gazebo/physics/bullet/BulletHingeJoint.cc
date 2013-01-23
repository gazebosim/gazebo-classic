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
/* Desc: A BulletHingeJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */
#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletHingeJoint::BulletHingeJoint(btDynamicsWorld *_world, BasePtr _parent)
    : HingeJoint<BulletJoint>(_parent)
{
  this->world = _world;
  this->btHinge = NULL;
  this->angleOffset = 0;
}

//////////////////////////////////////////////////
BulletHingeJoint::~BulletHingeJoint()
{
}

//////////////////////////////////////////////////
void BulletHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletHingeJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  HingeJoint<BulletJoint>::Attach(_one, _two);

  BulletLinkPtr bulletChildLink =
    boost::shared_static_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::shared_static_cast<BulletLink>(this->parentLink);

  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  math::Vector3 axis = axisElem->GetValueVector3("xyz");

  math::Vector3 pivotA, pivotB, axisA, axisB;
  math::Pose pose;

  // Compute the pivot point, based on the anchorPos
  pivotA = this->anchorPos;
  pivotB = this->anchorPos;
  if (this->parentLink)
  {
    pose = this->parentLink->GetWorldCoGPose();
    pivotA -= pose.pos;
    pivotA = pose.rot.RotateVectorReverse(pivotA);
    axisA = pose.rot.RotateVectorReverse(axis);
  }
  if (this->childLink)
  {
    pose = this->childLink->GetWorldCoGPose();
    pivotB -= pose.pos;
    pivotB = pose.rot.RotateVectorReverse(pivotB);
    axisB = pose.rot.RotateVectorReverse(axis);
  }

  axisA = axisA.Round();
  axisB = axisB.Round();


  if (bulletChildLink && bulletParentLink)
    this->btHinge = new btHingeConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        btVector3(pivotA.x, pivotA.y, pivotA.z),
        btVector3(pivotB.x, pivotB.y, pivotB.z),
        btVector3(axisA.x, axisA.y, axisA.z),
        btVector3(axisB.x, axisB.y, axisB.z));
  else if (bulletChildLink)
    this->btHinge = new btHingeConstraint(
        *bulletChildLink->GetBulletLink(),
        btVector3(pivotB.x, pivotB.y, pivotB.z),
        btVector3(axisB.x, axisB.y, axisB.z));
  else if (bulletParentLink)
    this->btHinge = new btHingeConstraint(
        *bulletParentLink->GetBulletLink(),
        btVector3(pivotA.x, pivotA.y, pivotA.z),
        btVector3(axisA.x, axisA.y, axisA.z));
  else
  {
    gzerr << "joint without links\n";
    gzthrow("joint without links\n");
  }

  this->constraint = this->btHinge;

  this->angleOffset = this->btHinge->getHingeAngle();
  // double angle = this->btHinge->getHingeAngle();
  // this->btHinge->setLimit(angle - .4, angle + .4);
  // Add the joint to the world
  this->world->addConstraint(this->btHinge, true);

  // Allows access to impulse
  this->btHinge->enableFeedback(true);
}

//////////////////////////////////////////////////
math::Vector3 BulletHingeJoint::GetAnchor(int /*_index*/) const
{
  btTransform trans = this->btHinge->getAFrame();
  trans.getOrigin() +=
    this->btHinge->getRigidBodyA().getCenterOfMassTransform().getOrigin();
  return math::Vector3(trans.getOrigin().getX(),
      trans.getOrigin().getY(), trans.getOrigin().getZ());
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetAnchor(int /*_index*/,
                                 const math::Vector3 &/*_anchor*/)
{
  // The anchor (pivot in Bullet lingo), can only be set on creation
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3 vec(_axis.x, _axis.y, _axis.z);
  ((btHingeConstraint*)this->btHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle BulletHingeJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->btHinge != NULL)
    result = this->btHinge->getHingeAngle() - this->angleOffset;
  else
    gzerr << "btHinge does not exist, returning default angle\n";
  return result;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  // this->btHinge->enableAngularMotor(true, -_angle,
  // this->GetMaxForce(_index));
}

//////////////////////////////////////////////////
double BulletHingeJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetMaxForce(int /*_index*/, double _t)
{
  this->btHinge->setMaxMotorImpulse(_t);
}

//////////////////////////////////////////////////
double BulletHingeJoint::GetMaxForce(int /*_index*/)
{
  return this->btHinge->getMaxMotorImpulse();
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetForce(int /*_index*/, double _torque)
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
double BulletHingeJoint::GetForce(int /*_index*/)
{
  return this->btHinge->getAppliedImpulse();
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetHighStop(int /*_index*/,
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
void BulletHingeJoint::SetLowStop(int /*_index*/,
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
math::Angle BulletHingeJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;

  if (this->btHinge)
    result = this->btHinge->getUpperLimit();
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////
math::Angle BulletHingeJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  if (this->btHinge)
    result = this->btHinge->getLowerLimit();
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletHingeJoint::GetGlobalAxis(int /*_index*/) const
{
  math::Vector3 result;
  if (this->btHinge)
  {
    btVector3 vec =
      btHinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
      btHinge->getFrameOffsetA().getBasis().getColumn(2);
    result.x = vec.getX();
    result.y = vec.getY();
    result.z = vec.getZ();
  }
  else
    gzerr << "btHinge does not exist, returning fake axis\n";
  return result;
}
