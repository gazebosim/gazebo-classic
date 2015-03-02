/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != NULL, "Joint axis sdf member is NULL");
  {
    sdf::ElementPtr limitElem;
    limitElem = this->sdf->GetElement("axis")->GetElement("limit");
    this->bulletSlider->setLowerLinLimit(limitElem->Get<double>("lower"));
    this->bulletSlider->setUpperLinLimit(limitElem->Get<double>("upper"));
  }

  // Set Joint friction here in Init, since the bullet data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

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
double BulletSliderJoint::GetVelocity(unsigned int /*_index*/) const
{
  double result = 0;
  math::Vector3 globalAxis = this->GetGlobalAxis(0);
  if (this->childLink)
    result += globalAxis.Dot(this->childLink->GetWorldLinearVel());
  if (this->parentLink)
    result -= globalAxis.Dot(this->parentLink->GetWorldLinearVel());
  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (!this->bulletSlider)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis
    math::Quaternion axisFrame = this->GetAxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetDamping(unsigned int /*index*/,
    const double _damping)
{
  /// \TODO: special case bullet specific linear damping, this needs testing.
  if (this->bulletSlider)
    this->bulletSlider->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
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
bool BulletSliderJoint::SetHighStop(unsigned int /*_index*/,
                                    const math::Angle &_angle)
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
bool BulletSliderJoint::SetLowStop(unsigned int /*_index*/,
                                   const math::Angle &_angle)
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
math::Angle BulletSliderJoint::GetHighStop(unsigned int /*_index*/)
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getUpperLinLimit();
  else
    gzlog << "Joint must be created before getting high stop\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetLowStop(unsigned int /*_index*/)
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getLowerLinLimit();
  else
    gzlog << "Joint must be created before getting low stop\n";
  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetMaxForce(unsigned int /*_index*/, double _force)
{
  if (this->bulletSlider)
    this->bulletSlider->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetMaxForce(unsigned int /*_index*/)
{
  double result = 0;
  if (this->bulletSlider)
    result = this->bulletSlider->getMaxLinMotorForce();
  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletSliderJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  math::Vector3 result = this->initialWorldAxis;

  if (this->bulletSlider)
  {
    // bullet uses x-axis for slider
    btVector3 vec =
      this->bulletSlider->getRigidBodyA().getCenterOfMassTransform().getBasis()
      * this->bulletSlider->getFrameOffsetA().getBasis().getColumn(0);
    result = BulletTypes::ConvertVector3(vec);
  }

  return result;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetAngleImpl(unsigned int _index) const
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "Invalid axis index [" << _index << "]" << std::endl;
    return math::Angle();
  }

  // The getLinearPos function seems to be off by one time-step
  // https://github.com/bulletphysics/bullet3/issues/239
  // if (this->bulletSlider)
  //   result = this->bulletSlider->getLinearPos();
  // else
  //   gzlog << "bulletSlider does not exist, returning default position\n";

  // Compute slider angle from gazebo's cached poses instead
  math::Vector3 offset = this->GetWorldPose().pos
                 - this->GetParentWorldPose().pos;
  math::Vector3 axis = this->GetGlobalAxis(_index);
  math::Pose poseParent = this->GetWorldPose();
  return math::Angle(axis.Dot(offset));
}

//////////////////////////////////////////////////
bool BulletSliderJoint::SetParam(const std::string &_key,
    unsigned int _index,
    const boost::any &_value)
{
  if (_index >= this->GetAngleCount())
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
        // there is an upstream bug in bullet 2.82 and earlier
        // the maxLinMotorForce parameter is inadvertently divided
        // by timestep when attempting to compute an impulse in
        // the bullet solver.
        // https://github.com/bulletphysics/bullet3/pull/328
        // As a workaround, multiply the desired friction
        // parameter by dt^2 when setting
        double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
        this->bulletSlider->setMaxLinMotorForce(
          dt*dt * boost::any_cast<double>(_value));
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
double BulletSliderJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  if (_key == "friction")
  {
    if (this->bulletSlider)
    {
      // there is an upstream bug in bullet 2.82 and earlier
      // the maxLinMotorForce parameter is inadvertently divided
      // by timestep when attempting to compute an impulse in
      // the bullet solver.
      // https://github.com/bulletphysics/bullet3/pull/328
      // As a workaround, divide the desired friction
      // parameter by dt^2 when getting
      double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
      return this->bulletSlider->getMaxLinMotorForce() / (dt*dt);
    }
    else
    {
      gzerr << "Joint must be created before getting " << _key << std::endl;
      return 0.0;
    }
  }
  return BulletJoint::GetParam(_key, _index);
}
