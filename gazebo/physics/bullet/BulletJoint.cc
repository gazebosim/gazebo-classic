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
#include <functional>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletJointPrivate.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletJoint::BulletJoint(BasePtr _parent)
: Joint(*new BulletJointPrivate, _parent),
  bulletJointDPtr(static_cast<BulletJointPrivate*>(this->jointDPtr))
{
  this->bulletJointDPtr->constraint = nullptr;
  this->bulletJointDPtr->bulletWorld = nullptr;
  this->bulletJointDPtr->feedback = nullptr;
  this->bulletJointDPtr->stiffnessDampingInitialized = false;
  this->bulletJointDPtr->forceApplied[0] = 0;
  this->bulletJointDPtr->forceApplied[1] = 0;
}

//////////////////////////////////////////////////
BulletJoint::~BulletJoint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void BulletJoint::Fini()
{
  if (this->bulletJointDPtr->constraint && this->bulletJointDPtr->bulletWorld)
  {
    this->bulletJointDPtr->bulletWorld->removeConstraint(
        this->bulletJointDPtr->constraint);
    delete this->bulletJointDPtr->constraint;
  }
  this->bulletJointDPtr->constraint = nullptr;
  this->bulletJointDPtr->bulletWorld = nullptr;

  if (this->bulletJointDPtr->feedback)
    delete this->bulletJointDPtr->feedback;
  this->bulletJointDPtr->feedback = nullptr;

  Joint::Fini();
}

//////////////////////////////////////////////////
void BulletJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->bulletJointDPtr->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->bulletJointDPtr->sdf->GetElement("axis");
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

  if (this->bulletJointDPtr->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->bulletJointDPtr->sdf->GetElement("axis");
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
  return this->JointLink(_index);
}

//////////////////////////////////////////////////
LinkPtr BulletJoint::JointLink(const unsigned int _index) const
{
  LinkPtr result;

  if (this->bulletJointDPtr->constraint == nullptr)
  {
    gzerr << "Attach bodies to the joint first";
    return result;
  }

  if (_index == 0 || _index == 1)
  {
    BulletLinkPtr bulletLink1 =
      std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->childLink);

    BulletLinkPtr bulletLink2 =
      std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->parentLink);

    btRigidBody rigidLink = this->bulletJointDPtr->constraint->getRigidBodyA();

    if (bulletLink1 && rigidLink.getUserPointer() == bulletLink1.get())
      result = this->bulletJointDPtr->childLink;
    else if (bulletLink2)
      result = this->bulletJointDPtr->parentLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool BulletJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  return this->bulletJointDPtr->constraint &&
    ((this->bulletJointDPtr->childLink.get() == _one.get() &&
      this->bulletJointDPtr->parentLink.get() == _two.get()) ||
     (this->bulletJointDPtr->childLink.get() == _two.get() &&
      this->bulletJointDPtr->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void BulletJoint::Detach()
{
  this->applyDamping.reset();

  this->bulletJointDPtr->childLink.reset();
  this->bulletJointDPtr->parentLink.reset();
  if (this->bulletJointDPtr->constraint && this->bulletJointDPtr->bulletWorld)
  {
    this->bulletJointDPtr->bulletWorld->removeConstraint(
        this->bulletJointDPtr->constraint);
  }
  delete this->bulletJointDPtr->constraint;
  this->bulletJointDPtr->constraint = nullptr;
}

//////////////////////////////////////////////////
void BulletJoint::SetProvideFeedback(const bool _enable)
{
  Joint::SetProvideFeedback(_enable);

  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void BulletJoint::CacheForceTorque()
{
  if (!this->bulletJointDPtr->provideFeedback)
    return;

  // caching force torque for the joint
  // if cached, GetForceTorque should use this value
  // this->bulletJointDPtr->wrench
  this->bulletJointDPtr->wrench.body2Force = BulletTypes::ConvertVector3Ign(
                      this->bulletJointDPtr->feedback->m_appliedForceBodyA);
  this->bulletJointDPtr->wrench.body2Torque = BulletTypes::ConvertVector3Ign(
                      this->bulletJointDPtr->feedback->m_appliedTorqueBodyA);
  this->bulletJointDPtr->wrench.body1Force = BulletTypes::ConvertVector3Ign(
                      this->bulletJointDPtr->feedback->m_appliedForceBodyB);
  this->bulletJointDPtr->wrench.body1Torque = BulletTypes::ConvertVector3Ign(
                      this->bulletJointDPtr->feedback->m_appliedTorqueBodyB);
  // gzerr << "   " << this->GetName()
  //       << " : " << this->bulletJointDPtr->wrench.body1Force
  //       << " : " << this->bulletJointDPtr->wrench.body1Torque
  //       << " : " << this->bulletJointDPtr->wrench.body2Force
  //       << " : " << this->bulletJointDPtr->wrench.body2Torque
  //       << "\n";

  // get force applied through SetForce
  physics::JointWrench wrenchAppliedWorld;
  if (this->HasType(physics::Base::HINGE_JOINT))
  {
    // rotate force into child link frame
    // GetLocalAxis is the axis specified in parent link frame!!!
    wrenchAppliedWorld.body2Torque = this->Force(0u) * this->LocalAxis(0u);

    // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
    //       << "] axis [" << this->GetLocalAxis(0u)
    //       << "]\n";

    wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
  }
  else if (this->HasType(physics::Base::SLIDER_JOINT))
  {
    // rotate force into child link frame
    wrenchAppliedWorld.body2Force = this->Force(0u) * this->LocalAxis(0u);
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
  if (this->bulletJointDPtr->childLink)
  {
    ignition::math::Pose3d childPose =
      this->bulletJointDPtr->childLink->WorldPose();

    // convert torque from about child CG to joint anchor location
    // cg position specified in child link frame
    ignition::math::Pose3d cgPose =
      this->bulletJointDPtr->childLink->Inertial()->Pose();

    // anchorPose location of joint in child frame
    // childMomentArm: from child CG to joint location in child link frame
    // moment arm rotated into world frame (given feedback is in world frame)
    ignition::math::Vector3d childMomentArm = childPose.Rot().RotateVector(
        (this->bulletJointDPtr->anchorPose -
         ignition::math::Pose3d(cgPose.Pos(),
           ignition::math::Quaterniond())).Pos());

    // gzerr << "anchor [" << anchorPose
    //       << "] iarm[" << this->childLink->GetInertial()->GetPose().pos
    //       << "] childMomentArm[" << childMomentArm
    //       << "] f1[" << this->bulletJointDPtr->wrench.body2Force
    //       << "] t1[" << this->bulletJointDPtr->wrench.body2Torque
    //       << "] fxp[" << this->bulletJointDPtr->wrench.body2Force.Cross(
    //       childMomentArm)
    //       << "]\n";

    this->bulletJointDPtr->wrench.body2Torque +=
      this->bulletJointDPtr->wrench.body2Force.Cross(childMomentArm);

    // rotate resulting body2Force in world frame into link frame
    this->bulletJointDPtr->wrench.body2Force =
      childPose.Rot().RotateVectorReverse(
          -this->bulletJointDPtr->wrench.body2Force);

    // rotate resulting body2Torque in world frame into link frame
    this->bulletJointDPtr->wrench.body2Torque =
      childPose.Rot().RotateVectorReverse(
          -this->bulletJointDPtr->wrench.body2Torque);
  }

  // convert torque from about parent CG to joint anchor location
  if (this->bulletJointDPtr->parentLink)
  {
    // get child pose, or it's the inertial world if childLink is nullptr
    ignition::math::Pose3d childPose;

    if (this->bulletJointDPtr->childLink)
      childPose = this->bulletJointDPtr->childLink->WorldPose();

    ignition::math::Pose3d parentPose =
      this->bulletJointDPtr->parentLink->WorldPose();

    // if parent link exists, convert torque from about parent
    // CG to joint anchor location

    // parent cg specified in parent link frame
    ignition::math::Pose3d cgPose =
      this->bulletJointDPtr->parentLink->Inertial()->Pose();

    // get parent CG pose in child link frame
    ignition::math::Pose3d parentCGInChildLink =
      ignition::math::Pose3d(cgPose.Pos(), ignition::math::Quaterniond()) -
      (childPose - parentPose);

    // anchor location in parent CG frame
    // this is the moment arm, but it's in parent CG frame, we need
    // to convert it into world frame
    ignition::math::Pose3d anchorInParendCGFrame =
      this->bulletJointDPtr->anchorPose - parentCGInChildLink;

    // paretnCGFrame in world frame
    ignition::math::Pose3d parentCGInWorld = cgPose + parentPose;

    // rotate momeent arms into world frame
    ignition::math::Vector3d parentMomentArm =
      parentCGInWorld.Rot().RotateVector(
          (this->bulletJointDPtr->anchorPose - parentCGInChildLink).Pos());

    // gzerr << "anchor [" << this->anchorPose
    //       << "] pcginc[" << parentCGInChildLink
    //       << "] iarm[" << cgPose
    //       << "] anc2pcg[" << this->anchorPose - parentCGInChildLink
    //       << "] parentMomentArm[" << parentMomentArm
    //       << "] f1[" << this->bulletJointDPtr->wrench.body1Force
    //       << "] t1[" << this->bulletJointDPtr->wrench.body1Torque
    //       << "] fxp[" << this->bulletJointDPtr->wrench.body1Force.Cross(
    //       parentMomentArm)
    //       << "]\n";

    this->bulletJointDPtr->wrench.body1Torque +=
      this->bulletJointDPtr->wrench.body1Force.Cross(parentMomentArm);

    // rotate resulting body1Force in world frame into link frame
    this->bulletJointDPtr->wrench.body1Force =
      parentPose.Rot().RotateVectorReverse(
          -this->bulletJointDPtr->wrench.body1Force);

    // rotate resulting body1Torque in world frame into link frame
    this->bulletJointDPtr->wrench.body1Torque =
      parentPose.Rot().RotateVectorReverse(
          -this->bulletJointDPtr->wrench.body1Torque);

    if (!this->bulletJointDPtr->childLink)
    {
      // if child link does not exist, use equal and opposite
      this->bulletJointDPtr->wrench.body2Force =
        -this->bulletJointDPtr->wrench.body1Force;
      this->bulletJointDPtr->wrench.body2Torque =
        -this->bulletJointDPtr->wrench.body1Torque;

      // force/torque are in parent link frame, transform them into
      // child link(world) frame.
      ignition::math::Pose3d parentToWorldTransform =
        this->bulletJointDPtr->parentLink->WorldPose();

      this->bulletJointDPtr->wrench.body1Force =
        parentToWorldTransform.Rot().RotateVector(
        this->bulletJointDPtr->wrench.body1Force);

      this->bulletJointDPtr->wrench.body1Torque =
        parentToWorldTransform.Rot().RotateVector(
        this->bulletJointDPtr->wrench.body1Torque);
    }
  }
  else
  {
    if (!this->bulletJointDPtr->childLink)
    {
      gzerr << "Joint [" << this->ScopedName()
            << "]: Both parent and child links are invalid, abort.\n";
      return;
    }
    else
    {
      // if parentLink does not exist, use equal opposite body1 wrench
      this->bulletJointDPtr->wrench.body1Force =
        -this->bulletJointDPtr->wrench.body2Force;
      this->bulletJointDPtr->wrench.body1Torque =
        -this->bulletJointDPtr->wrench.body2Torque;

      // force/torque are in child link frame, transform them into
      // parent link frame.  Here, parent link is world, so zero transform.
      ignition::math::Pose3d childToWorldTransform =
        this->bulletJointDPtr->childLink->WorldPose();

      this->bulletJointDPtr->wrench.body1Force =
        childToWorldTransform.Rot().RotateVector(
        this->bulletJointDPtr->wrench.body1Force);
      this->bulletJointDPtr->wrench.body1Torque =
        childToWorldTransform.Rot().RotateVector(
        this->bulletJointDPtr->wrench.body1Torque);
    }
  }
  this->bulletJointDPtr->wrench = this->bulletJointDPtr->wrench - wrenchAppliedWorld;
}

//////////////////////////////////////////////////
JointWrench BulletJoint::ForceTorque(const unsigned int /*_index*/) const
{
  GZ_ASSERT(this->bulletJointDPtr->constraint != nullptr,
      "constraint should be valid");
  return this->bulletJointDPtr->wrench;
}

//////////////////////////////////////////////////
void BulletJoint::SetupJointFeedback()
{
  if (this->bulletJointDPtr->provideFeedback)
  {
    if (this->bulletJointDPtr->feedback == nullptr)
    {
      this->bulletJointDPtr->feedback = new btJointFeedback;
      this->bulletJointDPtr->feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
      this->bulletJointDPtr->feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
      this->bulletJointDPtr->feedback->m_appliedTorqueBodyA =
        btVector3(0, 0, 0);
      this->bulletJointDPtr->feedback->m_appliedTorqueBodyB =
        btVector3(0, 0, 0);
    }

    if (this->bulletJointDPtr->constraint)
    {
      this->bulletJointDPtr->constraint->setJointFeedback(
          this->bulletJointDPtr->feedback);
    }
    else
    {
      gzerr << "Bullet Joint [" << this->Name() << "] ID is invalid\n";
    }
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetDamping(const unsigned int _index, const double _damping)
{
  if (_index < this->AngleCount())
  {
    this->SetStiffnessDamping(_index,
        this->bulletJointDPtr->stiffnessCoefficient[_index], _damping);
  }
  else
  {
     gzerr << "BulletJoint::SetDamping: index[" << _index
           << "] is out of bounds (AngleCount() = "
           << this->AngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetStiffness(const unsigned int _index,
    const double _stiffness)
{
  if (_index < this->AngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->bulletJointDPtr->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "BulletJoint::SetStiffness: index[" << _index
           << "] is out of bounds (AngleCount() = "
           << this->AngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetStiffnessDamping(const unsigned int _index,
  const double _stiffness, const double _damping, const double _reference)
{
  if (_index < this->AngleCount())
  {
    this->bulletJointDPtr->stiffnessCoefficient[_index] = _stiffness;
    this->bulletJointDPtr->dissipationCoefficient[_index] = _damping;
    this->bulletJointDPtr->springReferencePosition[_index] = _reference;

    /// \TODO: this check might not be needed?  attaching an object to a static
    /// body should not affect damping application.
    bool parentStatic = this->Parent() ? this->Parent()->IsStatic() : false;
    bool childStatic = this->Child() ? this->Child()->IsStatic() : false;

    if (!this->bulletJointDPtr->stiffnessDampingInitialized)
    {
      if (!parentStatic && !childStatic)
      {
        this->bulletJointDPtr->applyDamping =
          physics::Joint::ConnectJointUpdate(
              std::bind(&BulletJoint::ApplyStiffnessDamping, this));
        this->bulletJointDPtr->stiffnessDampingInitialized = true;
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->Name()
               << "] is not initialized because either parent[" << parentStatic
               << "] or child[" << childStatic << "] is static.\n";
      }
    }
  }
  else
    gzerr << "SetStiffnessDamping _index too large.\n";
}

//////////////////////////////////////////////////
void BulletJoint::SetForce(const unsigned int _index,
    const double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->bulletJointDPtr->childLink)
    this->bulletJointDPtr->childLink->SetEnabled(true);
  if (this->bulletJointDPtr->parentLink)
    this->bulletJointDPtr->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void BulletJoint::SaveForce(const unsigned int _index, const double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->AngleCount())
  {
    if (this->bulletJointDPtr->forceAppliedTime < this->World()->GetSimTime())
    {
      // reset forces if time step is new
      this->bulletJointDPtr->forceAppliedTime = this->World()->GetSimTime();
      this->bulletJointDPtr->forceApplied[0] =
        this->bulletJointDPtr->forceApplied[1] = 0;
    }

    this->bulletJointDPtr->forceApplied[_index] += _force;
  }
  else
  {
    gzerr << "Something's wrong, joint [" << this->Name()
          << "] index [" << _index
          << "] out of range.\n";
  }
}

//////////////////////////////////////////////////
double BulletJoint::Force(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->bulletJointDPtr->forceApplied[_index];
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
  for (unsigned int i = 0; i < this->AngleCount(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce =
      -fabs(this->bulletJointDPtr->dissipationCoefficient[i]) *
      this->Velocity(i);

    double springForce = this->bulletJointDPtr->stiffnessCoefficient[i] *
      (this->bulletJointDPtr->springReferencePosition[i] -
       this->Angle(i).Radian());

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_anchor*/)
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
    const unsigned int /*_index*/,
    const boost::any &/*_value*/)
{
  gzdbg << "Not implement in Bullet\n";
  return false;
}

//////////////////////////////////////////////////
double BulletJoint::Param(const std::string &_key,
    const unsigned int _index) const
{
  return Joint::Param(_key, _index);
}

//////////////////////////////////////////////////
ignition::math::Angle BulletJoint::HighStop(const unsigned int _index) const
{
  return this->UpperLimit(_index);
}

//////////////////////////////////////////////////
ignition::math::Angle BulletJoint::LowStop(const unsigned int _index) const
{
  return this->LowerLimit(_index);
}

//////////////////////////////////////////////////
bool BulletJoint::SetPosition(const unsigned int _index, const double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}
