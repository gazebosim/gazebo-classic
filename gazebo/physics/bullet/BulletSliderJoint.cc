/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/BulletJointPrivate.hh"
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
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld = _world;
  this->bulletSlider = nullptr;
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
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->childLink);
  BulletLinkPtr bulletParentLink =
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->parentLink);

  // Get axis unit vector (expressed in world frame).
  ignition::math::Vector3d axis = this->initialWorldAxis;

  if (axis == ignition::math::Vector3d::Zero)
  {
    gzerr << "axis must have non-zero length, resetting to 0 0 1\n";
    axis.Set(0, 0, 1);
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  ignition::math::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  ignition::math::Pose3d pose;
  btTransform frameParent, frameChild;
  btVector3 axis2, axis3;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->bulletJointDPtr->anchorPos;
  pivotChild = this->bulletJointDPtr->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->bulletJointDPtr->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->bulletJointDPtr->parentLink->WorldCoGPose();

    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.Pos();

    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    frameParent.setOrigin(BulletTypes::ConvertVector3(pivotParent));
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();

    // The following math is based on btHingeConstraint.cpp:95-115
    btPlaneSpace1(BulletTypes::ConvertVector3(axisParent), axis2, axis3);

    frameParent.getBasis().setValue(
      axisParent.X(), axis2.x(), axis3.x(),
      axisParent.Y(), axis2.y(), axis3.y(),
      axisParent.Z(), axis2.z(), axis3.z());
  }

  // Check if childLink exists. If not, the child will be the world.
  if (this->bulletJointDPtr->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->bulletJointDPtr->childLink->WorldCoGPose();

    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.Pos();

    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
    frameChild.setOrigin(BulletTypes::ConvertVector3(pivotChild));
    axisChild = pose.Rot().RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();

    // The following math is based on btHingeConstraint.cpp:95-115
    btPlaneSpace1(BulletTypes::ConvertVector3(axisChild), axis2, axis3);

    frameChild.getBasis().setValue(
      axisChild.X(), axis2.x(), axis3.x(),
      axisChild.Y(), axis2.y(), axis3.y(),
      axisChild.Z(), axis2.z(), axis3.z());
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletParentLink->BtLink(),
        *bulletChildLink->BtLink(),
        frameParent, frameChild, true);
  }

  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletChildLink->BtLink(), frameChild, true);
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletParentLink->BtLink(), frameParent, true);
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "joint without links\n";
    return;
  }

  if (!this->bulletSlider)
  {
    gzerr << "unable to create bullet slider joint\n";
    return;
  }

  // btSliderConstraint has 2 degrees-of-freedom (like a piston)
  // so disable the rotation.
  this->bulletSlider->setLowerAngLimit(0.0);
  this->bulletSlider->setUpperAngLimit(0.0);

  // Apply joint translation limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->bulletJointDPtr->sdf != nullptr, "Joint sdf member is null");
  sdf::ElementPtr axisElem = this->bulletJointDPtr->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != nullptr, "Joint axis sdf member is null");
  {
    sdf::ElementPtr limitElem;
    limitElem =
      this->bulletJointDPtr->sdf->GetElement("axis")->GetElement("limit");
    this->bulletSlider->setLowerLinLimit(limitElem->Get<double>("lower"));
    this->bulletSlider->setUpperLinLimit(limitElem->Get<double>("upper"));
  }

  // Set Joint friction here in Init, since the bullet data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

  this->bulletJointDPtr->constraint = this->bulletSlider;

  // Add the joint to the world
  GZ_ASSERT(this->bulletJointDPtr->bulletWorld,
      "bullet world pointer is null");

  this->bulletJointDPtr->bulletWorld->addConstraint(this->bulletSlider, true);

  // Allows access to impulse
  this->bulletJointDPtr->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
double BulletSliderJoint::Velocity(const unsigned int /*_index*/) const
{
  double result = 0;
  ignition::math::Vector3d globalAxis = this->GlobalAxis(0);

  if (this->bulletJointDPtr->childLink)
  {
    result += globalAxis.Dot(
        this->bulletJointDPtr->childLink->WorldLinearVel());
  }

  if (this->bulletJointDPtr->parentLink)
  {
    result -= globalAxis.Dot(
        this->bulletJointDPtr->parentLink->WorldLinearVel());
  }

  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetVelocity(const unsigned int _index,
    const double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (!this->bulletSlider)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis
    ignition::math::Quaterniond axisFrame = this->AxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetDamping(const unsigned int /*index*/,
    const double _damping)
{
  /// \TODO: special case bullet specific linear damping, this needs testing.
  if (this->bulletSlider)
    this->bulletSlider->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetForceImpl(
    const unsigned int /*_index*/, const double _effort)
{
  if (this->bulletSlider && this->bulletJointDPtr->constraint)
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
    this->bulletJointDPtr->constraint->getRigidBodyA().applyCentralForce(
        -hingeForceA);

    this->bulletJointDPtr->constraint->getRigidBodyB().applyCentralForce(
        hingeForceB);
  }
}

//////////////////////////////////////////////////
bool BulletSliderJoint::SetHighStop(const unsigned int /*_index*/,
                                    const ignition::math::Angle &_angle)
{
  Joint::SetHighStop(0, _angle);
  if (this->bulletSlider)
  {
    this->bulletSlider->setUpperLinLimit(_angle.Radian());
    return true;
  }
  else
  {
    gzlog << "bulletSlider not yet created.\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool BulletSliderJoint::SetLowStop(const unsigned int /*_index*/,
                                   const ignition::math::Angle &_angle)
{
  Joint::SetLowStop(0, _angle);
  if (this->bulletSlider)
  {
    this->bulletSlider->setLowerLinLimit(_angle.Radian());
    return true;
  }
  else
  {
    gzlog << "bulletSlider not yet created.\n";
    return false;
  }
}

//////////////////////////////////////////////////
ignition::math::Angle BulletSliderJoint::HighStop(
    const unsigned int /*_index*/)
{
  ignition::math::Angle result;

  if (this->bulletSlider)
    result = this->bulletSlider->getUpperLinLimit();
  else
    gzlog << "Joint must be created before getting high stop\n";

  return result;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletSliderJoint::LowStop(
    const unsigned int /*_index*/) const
{
  ignition::math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getLowerLinLimit();
  else
    gzlog << "Joint must be created before getting low stop\n";
  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletSliderJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  ignition::math::Vector3d result = this->initialWorldAxis;

  if (this->bulletSlider)
  {
    // bullet uses x-axis for slider
    btVector3 vec =
      this->bulletSlider->getRigidBodyA().getCenterOfMassTransform().getBasis()
      * this->bulletSlider->getFrameOffsetA().getBasis().getColumn(0);
    result = BulletTypes::ConvertVector3Ign(vec);
  }

  return result;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletSliderJoint::AngleImpl(
    const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid axis index [" << _index << "]" << std::endl;
    return ignition::math::Angle::Zero;
  }

  // The getLinearPos function seems to be off by one time-step
  // https://github.com/bulletphysics/bullet3/issues/239
  // if (this->bulletSlider)
  //   result = this->bulletSlider->getLinearPos();
  // else
  //   gzlog << "bulletSlider does not exist, returning default position\n";

  // Compute slider angle from gazebo's cached poses instead
  ignition::math::Vector3d offset = this->WorldPose().Pos()
    - this->ParentWorldPose().Pos();

  ignition::math::Vector3d axis = this->GlobalAxis(_index);
  ignition::math::Pose3d poseParent = this->WorldPose();

  return ignition::math::Angle(axis.Dot(offset));
}

//////////////////////////////////////////////////
bool BulletSliderJoint::SetParam(const std::string &_key,
    const unsigned int _index,
    const boost::any &_value)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return false;
  }

  try
  {
    if (_key == "friction")
    {
      if (this->bulletSlider)
      {
        this->bulletSlider->setPoweredLinMotor(true);
        this->bulletSlider->setTargetLinMotorVelocity(0.0);
        double value = boost::any_cast<double>(_value);
#ifndef LIBBULLET_VERSION_GT_282
        // there is an upstream bug in bullet 2.82 and earlier
        // the maxLinMotorForce parameter is inadvertently divided
        // by timestep when attempting to compute an impulse in
        // the bullet solver.
        // https://github.com/bulletphysics/bullet3/pull/328
        // As a workaround, multiply the desired friction
        // parameter by dt^2 when setting
        double dt = this->jointDPtr->world->GetPhysicsEngine()->MaxStepSize();
        value *= dt*dt;
#endif
        this->bulletSlider->setMaxLinMotorForce(value);
      }
      else
      {
        gzerr << "Joint must be created before setting " << _key << std::endl;
        return false;
      }
    }
    else
    {
      return BulletJoint::SetParam(_key, _index, _value);
    }
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "SetParam(" << _key << ")"
          << " boost any_cast error:" << e.what()
          << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
double BulletSliderJoint::Param(const std::string &_key,
    const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  if (_key == "friction")
  {
    if (this->bulletSlider)
    {
      double value = this->bulletSlider->getMaxLinMotorForce();
#ifndef LIBBULLET_VERSION_GT_282
      // there is an upstream bug in bullet 2.82 and earlier
      // the maxLinMotorForce parameter is inadvertently divided
      // by timestep when attempting to compute an impulse in
      // the bullet solver.
      // https://github.com/bulletphysics/bullet3/pull/328
      // As a workaround, divide the desired friction
      // parameter by dt^2 when getting
      double dt =
        this->bulletJointDPtr->world->GetPhysicsEngine()->MaxStepSize();
      value /= dt*dt;
#endif
      return value;
    }
    else
    {
      gzerr << "Joint must be created before getting " << _key << std::endl;
      return 0.0;
    }
  }

  return BulletJoint::Param(_key, _index);
}
