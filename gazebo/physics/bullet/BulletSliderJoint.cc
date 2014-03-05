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
/* Desc: A bullet slider or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletSliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletSliderJoint::BulletSliderJoint(btDynamicsWorld *_world, BasePtr _parent)
    : SliderJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
  this->bulletSlider = NULL;
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
void BulletSliderJoint::Init()
{
  SliderJoint<BulletJoint>::Init();

  BulletLinkPtr bulletChildLink =
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  // Get axis unit vector (expressed in world frame).
  math::Vector3 axis = this->initialWorldAxis;
  if (axis == math::Vector3::Zero)
  {
    gzerr << "axis must have non-zero length, resetting to 0 0 1\n";
    axis.Set(0, 0, 1);
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3 pivotParent, pivotChild, axisParent, axisChild;
  math::Pose pose;
  btTransform frameParent, frameChild;
  btVector3 axis2, axis3;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.rot.RotateVectorReverse(pivotParent);
    frameParent.setOrigin(BulletTypes::ConvertVector3(pivotParent));
    axisParent = pose.rot.RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
    // The following math is based on btHingeConstraint.cpp:95-115
    btPlaneSpace1(BulletTypes::ConvertVector3(axisParent), axis2, axis3);
    frameParent.getBasis().setValue(
      axisParent.x, axis2.x(), axis3.x(),
      axisParent.y, axis2.y(), axis3.y(),
      axisParent.z, axis2.z(), axis3.z());
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.rot.RotateVectorReverse(pivotChild);
    frameChild.setOrigin(BulletTypes::ConvertVector3(pivotChild));
    axisChild = pose.rot.RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();
    // The following math is based on btHingeConstraint.cpp:95-115
    btPlaneSpace1(BulletTypes::ConvertVector3(axisChild), axis2, axis3);
    frameChild.getBasis().setValue(
      axisChild.x, axis2.x(), axis3.x(),
      axisChild.y, axis2.y(), axis3.y(),
      axisChild.z, axis2.z(), axis3.z());
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        frameParent, frameChild, true);
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletChildLink->GetBulletLink(), frameChild, true);
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletParentLink->GetBulletLink(), frameParent, true);
  }
  // Throw an error if no links are given.
  else
  {
    gzthrow("joint without links\n");
  }

  if (!this->bulletSlider)
    gzthrow("unable to create bullet slider joint\n");

  // btSliderConstraint has 2 degrees-of-freedom (like a piston)
  // so disable the rotation.
  this->bulletSlider->setLowerAngLimit(0.0);
  this->bulletSlider->setUpperAngLimit(0.0);

  // Apply joint translation limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr limitElem;
  limitElem = this->sdf->GetElement("axis")->GetElement("limit");
  this->bulletSlider->setLowerLinLimit(limitElem->Get<double>("lower"));
  this->bulletSlider->setUpperLinLimit(limitElem->Get<double>("upper"));

  this->constraint = this->bulletSlider;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->bulletSlider, true);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetVelocity(int /*_index*/) const
{
  double result = 0;
  // I'm not sure this will work
  if (this->bulletSlider)
    result = this->bulletSlider->getTargetLinMotorVelocity();
  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetVelocity(int /*_index*/, double _angle)
{
  if (this->bulletSlider)
    this->bulletSlider->setTargetLinMotorVelocity(_angle);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetAxis(int /*_index*/, const math::Vector3 &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (!this->bulletSlider)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis

    /// \TODO: currently we assume joint axis is specified in model frame,
    /// this is incorrect, and should be corrected to be
    /// joint frame which is specified in child link frame.
    if (this->parentLink)
      this->initialWorldAxis =
        this->GetParent()->GetModel()->GetWorldPose().rot.RotateVector(_axis);
    else
      this->initialWorldAxis = _axis;
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetDamping(int /*index*/, const double _damping)
{
  if (this->bulletSlider)
    this->bulletSlider->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetForceImpl(int /*_index*/, double _effort)
{
  if (this->bulletSlider && this->constraint)
  {
    // x-axis of constraint frame
    btVector3 hingeAxisLocalA =
      this->bulletSlider->getFrameOffsetA().getBasis().getColumn(0);
    btVector3 hingeAxisLocalB =
      this->bulletSlider->getFrameOffsetB().getBasis().getColumn(0);

    btVector3 hingeAxisWorldA =
      this->bulletSlider->getRigidBodyA().getWorldTransform().getBasis() *
      hingeAxisLocalA;
    btVector3 hingeAxisWorldB =
      this->bulletSlider->getRigidBodyB().getWorldTransform().getBasis() *
      hingeAxisLocalB;

    btVector3 hingeForceA = _effort * hingeAxisWorldA;
    btVector3 hingeForceB = _effort * hingeAxisWorldB;

    // TODO: switch to applyForce and specify body-fixed offset
    this->constraint->getRigidBodyA().applyCentralForce(-hingeForceA);
    this->constraint->getRigidBodyB().applyCentralForce(hingeForceB);
  }
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetHighStop(int /*_index*/,
                                    const math::Angle &_angle)
{
  Joint::SetHighStop(0, _angle);
  if (this->bulletSlider)
    this->bulletSlider->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetLowStop(int /*_index*/,
                                   const math::Angle &_angle)
{
  Joint::SetLowStop(0, _angle);
  if (this->bulletSlider)
    this->bulletSlider->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getUpperLinLimit();
  else
    gzerr << "Joint must be created before getting high stop\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getLowerLinLimit();
  else
    gzerr << "Joint must be created before getting low stop\n";
  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetMaxForce(int /*_index*/, double _force)
{
  if (this->bulletSlider)
    this->bulletSlider->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetMaxForce(int /*_index*/)
{
  double result = 0;
  if (this->bulletSlider)
    result = this->bulletSlider->getMaxLinMotorForce();
  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletSliderJoint::GetGlobalAxis(int /*_index*/) const
{
  math::Vector3 result;
  if (this->bulletSlider)
  {
    // I have not verified the following math, though I based it on internal
    // bullet code at line 250 of btHingeConstraint.cpp
    btVector3 vec =
      this->bulletSlider->getRigidBodyA().getCenterOfMassTransform().getBasis()
      * this->bulletSlider->getFrameOffsetA().getBasis().getColumn(2);
    result = BulletTypes::ConvertVector3(vec);
  }
  else
    gzwarn << "bulletHinge does not exist, returning fake axis\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getLinearPos();
  else
    gzwarn << "bulletSlider does not exist, returning default position\n";
  return result;
}
