/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
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
  this->bulletWorld = _world;
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
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

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
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and inertial frame of parent.
    pose = this->parentLink->WorldInertialPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.Pos();
    // Rotate pivot offset and axis into body-fixed inertial frame of parent.
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and inertial frame of child.
    pose = this->childLink->WorldInertialPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.Pos();
    // Rotate pivot offset and axis into body-fixed inertial frame of child.
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
        *(bulletChildLink->GetBulletLink()),
        *(bulletParentLink->GetBulletLink()),
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
        *(bulletChildLink->GetBulletLink()),
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
        *(bulletParentLink->GetBulletLink()),
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
  this->constraint = this->bulletHinge;

  // Set angleOffset based on hinge angle at joint creation.
  // PositionImpl will report angles relative to this offset.
  this->angleOffset = this->PositionImpl(0);

  // Apply joint angle limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->sdf != nullptr, "Joint sdf member is null");
  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != nullptr, "Joint axis sdf member is null");
  {
    sdf::ElementPtr limitElem;
    limitElem = this->sdf->GetElement("axis")->GetElement("limit");
    this->bulletHinge->setLimit(
      this->angleOffset + limitElem->Get<double>("lower"),
      this->angleOffset + limitElem->Get<double>("upper"));
  }

  // Set Joint friction here in Init, since the bullet data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is null");
  this->bulletWorld->addConstraint(this->bulletHinge, true);

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
    auto axisFrame = this->AxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }

  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btVector3 vec(_axis.X(), _axis.Y(), _axis.Z());
  ((btHingeConstraint*)this->bulletHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
double BulletHingeJoint::PositionImpl(const unsigned int /*_index*/) const
{
  double result = ignition::math::NAN_D;
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
void BulletHingeJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double BulletHingeJoint::GetVelocity(unsigned int /*_index*/) const
{
  double result = 0;
  ignition::math::Vector3d globalAxis = this->GlobalAxis(0);
  if (this->childLink)
    result += globalAxis.Dot(this->childLink->WorldAngularVel());
  if (this->parentLink)
    result -= globalAxis.Dot(this->parentLink->WorldAngularVel());
  return result;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
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
void BulletHingeJoint::SetUpperLimit(const unsigned int /*_index*/,
                                     const double _limit)
{
  Joint::SetUpperLimit(0, _limit);
  if (this->bulletHinge)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    this->bulletHinge->setLimit(this->bulletHinge->getLowerLimit(),
                                this->angleOffset + _limit);
  }
  else
  {
    static bool notPrintedYet = true;
    if (notPrintedYet)
    {
      gzerr << "Joint must be created before calling SetUpperLimit\n";
      notPrintedYet = false;
    }
  }
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetLowerLimit(const unsigned int /*_index*/,
                                     const double _limit)
{
  Joint::SetLowerLimit(0, _limit);
  if (this->bulletHinge)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    this->bulletHinge->setLimit(this->angleOffset + _limit,
                                this->bulletHinge->getUpperLimit());
  }
  else
  {
    static bool notPrintedYet = true;
    if (notPrintedYet)
    {
      gzerr << "Joint must be created before calling SetLowerLimit\n";
      notPrintedYet = false;
    }
  }
}

//////////////////////////////////////////////////
double BulletHingeJoint::UpperLimit(const unsigned int /*_index*/) const
{
  // the bullet limits are wrapped to [-pi,pi]
  // https://github.com/bulletphysics/bullet3/issues/42
  // it's more accurate to return our cached value for limit
  return this->upperLimit[0];
}

//////////////////////////////////////////////////
double BulletHingeJoint::LowerLimit(const unsigned int /*_index*/) const
{
  // the bullet limits are wrapped to [-pi,pi]
  // https://github.com/bulletphysics/bullet3/issues/42
  // it's more accurate to return our cached value for limit
  return this->lowerLimit[0];
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
    unsigned int _index,
    const boost::any &_value)
{
  if (_index >= this->DOF())
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
        double dt = this->world->Physics()->GetMaxStepSize();
        this->bulletHinge->enableAngularMotor(true, 0.0,
          dt * boost::any_cast<double>(_value));
      }
      else
      {
        static bool notPrintedYet = true;
        if (notPrintedYet)
        {
          gzerr << "Joint must be created before setting " << _key << std::endl;
          notPrintedYet = false;
        }
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
double BulletHingeJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  if (_key == "friction")
  {
    if (this->bulletHinge)
    {
      double dt = this->world->Physics()->GetMaxStepSize();
      return this->bulletHinge->getMaxMotorImpulse() / dt;
    }
    else
    {
      static bool notPrintedYet = true;
      if (notPrintedYet)
      {
        gzerr << "Joint must be created before getting " << _key << std::endl;
        notPrintedYet = false;
      }
      return 0.0;
    }
  }
  return BulletJoint::GetParam(_key, _index);
}
