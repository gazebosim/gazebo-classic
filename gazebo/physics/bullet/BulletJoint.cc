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
/* Desc: The base Bullet joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include <boost/bind.hpp>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletJoint::BulletJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->constraint = nullptr;
  this->bulletWorld = nullptr;
  this->feedback = nullptr;
  this->stiffnessDampingInitialized = false;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
}

//////////////////////////////////////////////////
BulletJoint::~BulletJoint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void BulletJoint::Fini()
{
  if (this->constraint && this->bulletWorld)
  {
    this->bulletWorld->removeConstraint(this->constraint);
    delete this->constraint;
  }
  this->constraint = nullptr;
  this->bulletWorld = nullptr;

  if (this->feedback)
    delete this->feedback;
  this->feedback = nullptr;

  Joint::Fini();
}

//////////////////////////////////////////////////
void BulletJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }
}

//////////////////////////////////////////////////
void BulletJoint::Init()
{
  Joint::Init();
}

//////////////////////////////////////////////////
void BulletJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr BulletJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;

  if (this->constraint == nullptr)
    gzthrow("Attach bodies to the joint first");

  if (_index == 0 || _index == 1)
  {
    BulletLinkPtr bulletLink1 =
      boost::static_pointer_cast<BulletLink>(this->childLink);

    BulletLinkPtr bulletLink2 =
      boost::static_pointer_cast<BulletLink>(this->parentLink);

    btRigidBody rigidLink = this->constraint->getRigidBodyA();

    if (bulletLink1 && rigidLink.getUserPointer() == bulletLink1.get())
      result = this->childLink;
    else if (bulletLink2)
      result = this->parentLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool BulletJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  return this->constraint && ((this->childLink.get() == _one.get() &&
                               this->parentLink.get() == _two.get()) ||
                              (this->childLink.get() == _two.get() &&
                               this->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void BulletJoint::Detach()
{
  this->applyDamping.reset();

  this->childLink.reset();
  this->parentLink.reset();
  if (this->constraint && this->bulletWorld)
    this->bulletWorld->removeConstraint(this->constraint);
  delete this->constraint;
  this->constraint = nullptr;
}

//////////////////////////////////////////////////
void BulletJoint::SetProvideFeedback(bool _enable)
{
  Joint::SetProvideFeedback(_enable);

  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void BulletJoint::CacheForceTorque()
{
  if (!this->provideFeedback)
    return;

  // caching force torque for the joint
  // if cached, GetForceTorque should use this value
  // this->wrench
  this->wrench.body2Force = BulletTypes::ConvertVector3Ign(
                      this->feedback->m_appliedForceBodyA);
  this->wrench.body2Torque = BulletTypes::ConvertVector3Ign(
                      this->feedback->m_appliedTorqueBodyA);
  this->wrench.body1Force = BulletTypes::ConvertVector3Ign(
                      this->feedback->m_appliedForceBodyB);
  this->wrench.body1Torque = BulletTypes::ConvertVector3Ign(
                      this->feedback->m_appliedTorqueBodyB);
  // gzerr << "   " << this->GetName()
  //       << " : " << this->wrench.body1Force
  //       << " : " << this->wrench.body1Torque
  //       << " : " << this->wrench.body2Force
  //       << " : " << this->wrench.body2Torque
  //       << "\n";

  // get force applied through SetForce
  physics::JointWrench wrenchAppliedWorld;
  if (this->HasType(physics::Base::HINGE_JOINT))
  {
    // rotate force into child link frame
    // LocalAxis is the axis specified in parent link frame!!!
    wrenchAppliedWorld.body2Torque =
      this->GetForce(0u) * this->LocalAxis(0u);

    // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
    //       << "] axis [" << this->LocalAxis(0u)
    //       << "]\n";

    wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
  }
  else if (this->HasType(physics::Base::SLIDER_JOINT))
  {
    // rotate force into child link frame
    wrenchAppliedWorld.body2Force =
      this->GetForce(0u) * this->LocalAxis(0u);
    wrenchAppliedWorld.body1Force = -wrenchAppliedWorld.body2Force;
  }
  else
  {
    /// \TODO: implement for other joint types
    // note that for fixed joint no further modification is needed
    // gzerr << "force torque for joint type [" << this->GetType()
    //       << "] not implemented, returns false results!!\n";
  }

  // convert wrench from child cg location to child link frame
  if (this->childLink)
  {
    ignition::math::Pose3d childPose = this->childLink->WorldPose();

    // convert torque from about child CG to joint anchor location
    // cg position specified in child link frame
    ignition::math::Pose3d cgPose =
      this->childLink->GetInertial()->Pose();

    // anchorPose location of joint in child frame
    // childMomentArm: from child CG to joint location in child link frame
    // moment arm rotated into world frame (given feedback is in world frame)
    ignition::math::Vector3d childMomentArm = childPose.Rot().RotateVector(
      (this->anchorPose -
       ignition::math::Pose3d(cgPose.Pos(),
         ignition::math::Quaterniond::Identity)).Pos());

    // gzerr << "anchor [" << anchorPose
    //       << "] iarm[" << this->childLink->GetInertial()->GetPose().Pos()
    //       << "] childMomentArm[" << childMomentArm
    //       << "] f1[" << this->wrench.body2Force
    //       << "] t1[" << this->wrench.body2Torque
    //       << "] fxp[" << this->wrench.body2Force.Cross(childMomentArm)
    //       << "]\n";

    this->wrench.body2Torque += this->wrench.body2Force.Cross(childMomentArm);

    // rotate resulting body2Force in world frame into link frame
    this->wrench.body2Force = childPose.Rot().RotateVectorReverse(
      -this->wrench.body2Force);

    // rotate resulting body2Torque in world frame into link frame
    this->wrench.body2Torque = childPose.Rot().RotateVectorReverse(
      -this->wrench.body2Torque);
  }

  // convert torque from about parent CG to joint anchor location
  if (this->parentLink)
  {
    // get child pose, or it's the inertial world if childLink is nullptr
    ignition::math::Pose3d childPose;
    if (this->childLink)
      childPose = this->childLink->WorldPose();

    ignition::math::Pose3d parentPose = this->parentLink->WorldPose();

    // if parent link exists, convert torque from about parent
    // CG to joint anchor location

    // parent cg specified in parent link frame
    ignition::math::Pose3d cgPose =
      this->parentLink->GetInertial()->Pose();

    // get parent CG pose in child link frame
    ignition::math::Pose3d parentCGInChildLink =
      ignition::math::Pose3d(cgPose.Pos(),
          ignition::math::Quaterniond::Identity) - (childPose - parentPose);

    // paretnCGFrame in world frame
    ignition::math::Pose3d parentCGInWorld = cgPose + parentPose;

    // rotate momeent arms into world frame
    ignition::math::Vector3d parentMomentArm =
      parentCGInWorld.Rot().RotateVector(
          (this->anchorPose - parentCGInChildLink).Pos());

    // gzerr << "anchor [" << this->anchorPose
    //       << "] pcginc[" << parentCGInChildLink
    //       << "] iarm[" << cgPose
    //       << "] anc2pcg[" << this->anchorPose - parentCGInChildLink
    //       << "] parentMomentArm[" << parentMomentArm
    //       << "] f1[" << this->wrench.body1Force
    //       << "] t1[" << this->wrench.body1Torque
    //       << "] fxp[" << this->wrench.body1Force.Cross(parentMomentArm)
    //       << "]\n";

    this->wrench.body1Torque += this->wrench.body1Force.Cross(
        parentMomentArm);

    // rotate resulting body1Force in world frame into link frame
    this->wrench.body1Force = parentPose.Rot().RotateVectorReverse(
      -this->wrench.body1Force);

    // rotate resulting body1Torque in world frame into link frame
    this->wrench.body1Torque = parentPose.Rot().RotateVectorReverse(
      -this->wrench.body1Torque);

    if (!this->childLink)
    {
      // if child link does not exist, use equal and opposite
      this->wrench.body2Force = -this->wrench.body1Force;
      this->wrench.body2Torque = -this->wrench.body1Torque;

      // force/torque are in parent link frame, transform them into
      // child link(world) frame.
      auto parentToWorldTransform = this->parentLink->WorldPose();
      this->wrench.body1Force =
        parentToWorldTransform.Rot().RotateVector(
        this->wrench.body1Force);
      this->wrench.body1Torque =
        parentToWorldTransform.Rot().RotateVector(
        this->wrench.body1Torque);
    }
  }
  else
  {
    if (!this->childLink)
    {
      gzerr << "Joint [" << this->GetScopedName()
            << "]: Both parent and child links are invalid, abort.\n";
      return;
    }
    else
    {
      // if parentLink does not exist, use equal opposite body1 wrench
      this->wrench.body1Force = -this->wrench.body2Force;
      this->wrench.body1Torque = -this->wrench.body2Torque;

      // force/torque are in child link frame, transform them into
      // parent link frame.  Here, parent link is world, so zero transform.
      auto childToWorldTransform = this->childLink->WorldPose();
      this->wrench.body1Force =
        childToWorldTransform.Rot().RotateVector(
        this->wrench.body1Force);
      this->wrench.body1Torque =
        childToWorldTransform.Rot().RotateVector(
        this->wrench.body1Torque);
    }
  }
  this->wrench = this->wrench - wrenchAppliedWorld;
}

//////////////////////////////////////////////////
JointWrench BulletJoint::GetForceTorque(unsigned int /*_index*/)
{
  GZ_ASSERT(this->constraint != nullptr, "constraint should be valid");
  return this->wrench;
}

//////////////////////////////////////////////////
void BulletJoint::SetupJointFeedback()
{
  if (this->provideFeedback)
  {
    if (this->feedback == nullptr)
    {
      this->feedback = new btJointFeedback;
      this->feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
      this->feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
      this->feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
      this->feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);
    }

    if (this->constraint)
      this->constraint->setJointFeedback(this->feedback);
    else
      gzerr << "Bullet Joint [" << this->GetName() << "] ID is invalid\n";
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "BulletJoint::SetDamping: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "BulletJoint::SetStiffness: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->DOF())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    /// \TODO: this check might not be needed?  attaching an object to a static
    /// body should not affect damping application.
    bool parentStatic =
      this->GetParent() ? this->GetParent()->IsStatic() : false;
    bool childStatic =
      this->GetChild() ? this->GetChild()->IsStatic() : false;

    if (!this->stiffnessDampingInitialized)
    {
      if (!parentStatic && !childStatic)
      {
        this->applyDamping = physics::Joint::ConnectJointUpdate(
          boost::bind(&BulletJoint::ApplyStiffnessDamping, this));
        this->stiffnessDampingInitialized = true;
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->GetName()
               << "] is not initialized because either parent[" << parentStatic
               << "] or child[" << childStatic << "] is static.\n";
      }
    }
  }
  else
    gzerr << "SetStiffnessDamping _index too large.\n";
}

//////////////////////////////////////////////////
void BulletJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void BulletJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->DOF())
  {
    if (this->forceAppliedTime < this->GetWorld()->SimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->SimTime();
      this->forceApplied[0] = this->forceApplied[1] = 0;
    }

    this->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
double BulletJoint::GetForce(unsigned int _index)
{
  if (_index < this->DOF())
  {
    return this->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

//////////////////////////////////////////////////
void BulletJoint::ApplyStiffnessDamping()
{
  for (unsigned int i = 0; i < this->DOF(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = -fabs(this->dissipationCoefficient[i])
      * this->GetVelocity(i);

    double springForce = this->stiffnessCoefficient[i]
      * (this->springReferencePosition[i] - this->Position(i));

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d & /*_anchor*/)
{
  // nothing to do here for bullet.
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletJoint::Anchor(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implement in Bullet\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletJoint::LinkForce(
          const unsigned int /*_index*/) const
{
  gzerr << "Not implement in Bullet\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletJoint::LinkTorque(
          const unsigned int /*_index*/) const
{
  gzerr << "Not implement in Bullet\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
bool BulletJoint::SetParam(const std::string &/*_key*/,
    unsigned int /*_index*/,
    const boost::any &/*_value*/)
{
  gzdbg << "Not implement in Bullet\n";
  return false;
}

//////////////////////////////////////////////////
double BulletJoint::GetParam(const std::string &_key,
    unsigned int _index)
{
  return Joint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
bool BulletJoint::SetPosition(const unsigned int _index, const double _position,
                              const bool _preserveWorldVelocity)
{
  // The code inside this ifdef is only relevant for versions of bullet greater
  // than 2.82. Any versions earlier than that will be broken no matter what.
#ifdef LIBBULLET_VERSION_GT_282
  // The following code fixes issue 2430 without breaking ABI. The key is to
  // take relatively small steps towards a target position so that the
  // accumulated angle doesn't bug out and reset itself from receiving too large
  // of a change all at once.
  if (this->HasType(Base::HINGE_JOINT))
  {
    double currentAngle = this->Position(0);

    const double maxSupportedAngle = 1e12;
    if (   std::abs(currentAngle) > maxSupportedAngle
        || std::abs(_position) > maxSupportedAngle)
    {
      // If either the current or the target angle are absurdly large, we will
      // get permanently stuck in the while-loop below, because trying to
      // increment currentAngle by delta will (eventually) not be able to modify
      // its floating point value.
      //
      // We put this check here to make sure that we can't get permanently stuck
      // in the loop.
      gzerr << "For Bullet hinge joints, the function Joint::SetPosition(~) "
            << "does not support positions larger than [" << maxSupportedAngle
            << "] radians. Your joint currently has an angle of ["
            << currentAngle << "] radians, and you are requesting an angle of ["
            << _position << "] radians.\n";

      // We can still call Joint::SetPositionMaximal in the normal way to reset
      // this joint angle. The end result should put this joint angle within
      // the range of [-pi, pi]. That should at least bring it back to a sane
      // state, even if it's not exactly what the user asked for.
      Joint::SetPositionMaximal(_index, _position, _preserveWorldVelocity);

      return false;
    }

    // Based on the source code of Bullet, the largest position change that
    // shouldn't bug out is 0.3 radians, so we keep our changes a little bit
    // below that.
    const double delta = 0.25;

    do
    {
      if (std::abs(_position - currentAngle) > delta)
      {
        currentAngle += _position > currentAngle? delta : -delta;
      }
      else
      {
        currentAngle = _position;
      }

      if (!Joint::SetPositionMaximal(_index, currentAngle,
                                     _preserveWorldVelocity))
      {
        return false;
      }
    } while ( std::abs(currentAngle - _position) > 1e-16);

    return true;
  }
#endif

  return Joint::SetPositionMaximal(_index, _position, _preserveWorldVelocity);
}
