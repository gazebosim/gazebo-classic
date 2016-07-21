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
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletHingeJoint::BulletHingeJoint(btDynamicsWorld *_world, BasePtr _parent)
: HingeJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld = _world;
  this->bulletHinge = nullptr;
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
void BulletHingeJoint::Init()
{
  HingeJoint<BulletJoint>::Init();

  // Cast to BulletLink
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
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
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
    axisChild = pose.Rot().RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
#ifdef LIBBULLET_VERSION_GT_282
    this->bulletHinge = new btHingeAccumulatedAngleConstraint(
#else
    this->bulletHinge = new btHingeConstraint(
#endif
        *(bulletChildLink->BtLink()), *(bulletParentLink->BtLink()),
        BulletTypes::ConvertVector3(pivotChild),
        BulletTypes::ConvertVector3(pivotParent),
        BulletTypes::ConvertVector3(axisChild),
        BulletTypes::ConvertVector3(axisParent));
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
#ifdef LIBBULLET_VERSION_GT_282
    this->bulletHinge = new btHingeAccumulatedAngleConstraint(
#else
    this->bulletHinge = new btHingeConstraint(
#endif
        *(bulletChildLink->BtLink()),
        BulletTypes::ConvertVector3(pivotChild),
        BulletTypes::ConvertVector3(axisChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
#ifdef LIBBULLET_VERSION_GT_282
    this->bulletHinge = new btHingeAccumulatedAngleConstraint(
#else
    this->bulletHinge = new btHingeConstraint(
#endif
        *(bulletParentLink->BtLink()),
        BulletTypes::ConvertVector3(pivotParent),
        BulletTypes::ConvertVector3(axisParent));
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "unable to create bullet hinge without links.\n";
    return;
  }

  if (!this->bulletHinge)
  {
    gzerr << "unable to create bullet hinge constraint\n";
    return;
  }

  // Give parent class BulletJoint a pointer to this constraint.
  this->bulletJointDPtr->constraint = this->bulletHinge;

  // Set angleOffset based on hinge angle at joint creation.
  // GetAngleImpl will report angles relative to this offset.
  this->angleOffset = this->AngleImpl(0).Radian();

  // Apply joint angle limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->bulletJointDPtr->sdf != nullptr, "Joint sdf member is null");
  sdf::ElementPtr axisElem = this->bulletJointDPtr->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != null, "Joint axis sdf member is null");
  {
    sdf::ElementPtr limitElem;
    limitElem =
      this->bulletJointDPtr->sdf->GetElement("axis")->GetElement("limit");

    this->bulletHinge->setLimit(
      this->angleOffset + limitElem->Get<double>("lower"),
      this->angleOffset + limitElem->Get<double>("upper"));
  }

  // Set Joint friction here in Init, since the bullet data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

  // Add the joint to the world
  GZ_ASSERT(this->bulletJointDPtr->bulletWorld, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld->addConstraint(this->bulletHinge, true);

  // Allows access to impulse
  this->bulletHinge->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHingeJoint::Anchor(
    const unsigned int /*_index*/) const
{
  btTransform trans = this->bulletHinge->getAFrame();
  trans.getOrigin() +=
    this->bulletHinge->getRigidBodyA().getCenterOfMassTransform().getOrigin();

  return ignition::math::Vector3d(trans.getOrigin().getX(),
      trans.getOrigin().getY(), trans.getOrigin().getZ());
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (this->bulletHinge == nullptr)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis
    ignition::math::Quaterniond axisFrame = this->AxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }

  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btignition::math::Vector3d vec(_axis.x, _axis.y, _axis.z);
  ((btHingeConstraint*)this->bulletHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHingeJoint::AngleImpl(
    const unsigned int /*_index*/) const
{
  ignition::math::Angle result;

  if (this->bulletHinge)
  {
#ifdef LIBBULLET_VERSION_GT_282
    btHingeAccumulatedAngleConstraint* hinge =
      static_cast<btHingeAccumulatedAngleConstraint*>(this->bulletHinge);

    if (hinge)
    {
      result = hinge->getAccumulatedHingeAngle();
    }
    else
#endif
    {
      result = this->bulletHinge->getHingeAngle();
    }
    result -= this->angleOffset;
  }

  return result;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetVelocity(const unsigned int _index,
    const double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double BulletHingeJoint::Velocity(const unsigned int /*_index*/) const
{
  double result = 0;
  ignition::math::Vector3d globalAxis = this->GlobalAxis(0);

  if (this->bulletJointDPtr->childLink)
  {
    result += globalAxis.Dot(
        this->bulletJointDPtr->childLink->WorldAngularVel());
  }

  if (this->bulletJointDPtr->parentLink)
  {
    result -= globalAxis.Dot(
        this->bulletJointDPtr->parentLink->WorldAngularVel());
  }

  return result;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetForceImpl(const unsigned int /*_index*/,
    const double _effort)
{
  if (this->bulletHinge)
  {
    // z-axis of constraint frame
    btVector3 hingeAxisLocalA =
      this->bulletHinge->getFrameOffsetA().getBasis().getColumn(2);

    btVector3 hingeAxisLocalB =
      this->bulletHinge->getFrameOffsetB().getBasis().getColumn(2);

    btVector3 hingeAxisWorldA =
      this->bulletHinge->getRigidBodyA().getWorldTransform().getBasis() *
      hingeAxisLocalA;

    btVector3 hingeAxisWorldB =
      this->bulletHinge->getRigidBodyB().getWorldTransform().getBasis() *
      hingeAxisLocalB;

    btVector3 hingeTorqueA = _effort * hingeAxisWorldA;
    btVector3 hingeTorqueB = _effort * hingeAxisWorldB;

    this->bulletHinge->getRigidBodyA().applyTorque(hingeTorqueA);
    this->bulletHinge->getRigidBodyB().applyTorque(-hingeTorqueB);
  }
}

//////////////////////////////////////////////////
bool BulletHingeJoint::SetHighStop(const unsigned int /*_index*/,
    const ignition::math::Angle &_angle)
{
  Joint::SetHighStop(0, _angle);

  if (this->bulletHinge)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    this->bulletHinge->setLimit(this->bulletHinge->getLowerLimit(),
                                this->angleOffset + _angle.Radian());
    return true;
  }
  else
  {
    gzerr << "bulletHinge not yet created.\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool BulletHingeJoint::SetLowStop(const unsigned int /*_index*/,
    const ignition::math::Angle &_angle)
{
  Joint::SetLowStop(0, _angle);

  if (this->bulletHinge)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    this->bulletHinge->setLimit(this->angleOffset + _angle.Radian(),
                                this->bulletHinge->getUpperLimit());
    return true;
  }
  else
  {
    gzerr << "bulletHinge not yet created.\n";
    return false;
  }
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHingeJoint::HighStop(
    const unsigned int /*_index*/) const
{
  ignition::math::Angle result;

  if (this->bulletHinge)
    result = this->bulletHinge->getUpperLimit();
  else
    gzerr << "Joint must be created before getting high stop\n";

  return result;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletHingeJoint::LowStop(
    const unsigned int /*_index*/) const
{
  ignition::math::Angle result;
  if (this->bulletHinge)
    result = this->bulletHinge->getLowerLimit();
  else
    gzerr << "Joint must be created before getting low stop\n";

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletHingeJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  ignition::math::Vector3d result = this->initialWorldAxis;

  if (this->bulletHinge)
  {
    // I have not verified the following math, though I based it on internal
    // bullet code at line 250 of btHingeConstraint.cpp
    btVector3 vec =
      bulletHinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
      bulletHinge->getFrameOffsetA().getBasis().getColumn(2);
    result = BulletTypes::ConvertVector3Ign(vec);
  }

  return result;
}

//////////////////////////////////////////////////
bool BulletHingeJoint::SetParam(const std::string &_key,
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
      if (this->bulletHinge)
      {
        // enableAngularMotor takes max impulse as a parameter
        // instead of max force.
        // this means the friction will change when the step size changes.
        double dt =
          this->bulletJointDPtr->world->GetPhysicsEngine()->MaxStepSize();

        this->bulletHinge->enableAngularMotor(true, 0.0,
          dt * boost::any_cast<double>(_value));
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
double BulletHingeJoint::Param(const std::string &_key,
    const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  if (_key == "friction")
  {
    if (this->bulletHinge)
    {
      double dt =
        this->bulletJointDPtr->world->GetPhysicsEngine()->MaxStepSize();
      return this->bulletHinge->getMaxMotorImpulse() / dt;
    }
    else
    {
      gzerr << "Joint must be created before getting " << _key << std::endl;
      return 0.0;
    }
  }
  return BulletJoint::Param(_key, _index);
}
