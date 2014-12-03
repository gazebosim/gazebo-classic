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
/* Desc: The base Bullet joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include <string>

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
  this->constraint = NULL;
  this->bulletWorld = NULL;
  this->feedback = NULL;
  this->stiffnessDampingInitialized = false;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
}

//////////////////////////////////////////////////
BulletJoint::~BulletJoint()
{
  delete this->constraint;
  this->bulletWorld = NULL;
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

  if (this->constraint == NULL)
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
  this->childLink.reset();
  this->parentLink.reset();
  if (this->constraint && this->bulletWorld)
    this->bulletWorld->removeConstraint(this->constraint);
  delete this->constraint;
}

//////////////////////////////////////////////////
void BulletJoint::CacheForceTorque()
{
  if (!this->provideFeedback)
    return;

  // caching force torque for the joint
  // if cached, GetForceTorque should use this value
  // this->wrench
  this->wrench.body2Force = BulletTypes::ConvertVector3(
                      this->feedback->m_appliedForceBodyA);
  this->wrench.body2Torque = BulletTypes::ConvertVector3(
                      this->feedback->m_appliedTorqueBodyA);
  this->wrench.body1Force = BulletTypes::ConvertVector3(
                      this->feedback->m_appliedForceBodyB);
  this->wrench.body1Torque = BulletTypes::ConvertVector3(
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
    // GetLocalAxis is the axis specified in parent link frame!!!
    wrenchAppliedWorld.body2Torque =
      this->GetForce(0u) * this->GetLocalAxis(0u);

    // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
    //       << "] axis [" << this->GetLocalAxis(0u)
    //       << "]\n";

    wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
  }
  else if (this->HasType(physics::Base::SLIDER_JOINT))
  {
    // rotate force into child link frame
    wrenchAppliedWorld.body2Force =
      this->GetForce(0u) * this->GetLocalAxis(0u);
    wrenchAppliedWorld.body1Force = -wrenchAppliedWorld.body2Force;
  }
  else
  {
    /// \TODO: implement for other joint types
    // gzerr << "force torque for joint type [" << this->GetType()
    //       << "] not implemented, returns false results!!\n";
  }

  // convert wrench from child cg location to child link frame
  if (this->childLink)
  {
    math::Pose childPose = this->childLink->GetWorldPose();

    // convert torque from about child CG to joint anchor location
    // cg position specified in child link frame
    math::Pose cgPose = this->childLink->GetInertial()->GetPose();

    // anchorPose location of joint in child frame
    // childMomentArm: from child CG to joint location in child link frame
    // moment arm rotated into world frame (given feedback is in world frame)
    math::Vector3 childMomentArm = childPose.rot.RotateVector(
      (this->anchorPose - math::Pose(cgPose.pos, math::Quaternion())).pos);

    // gzerr << "anchor [" << anchorPose
    //       << "] iarm[" << this->childLink->GetInertial()->GetPose().pos
    //       << "] childMomentArm[" << childMomentArm
    //       << "] f1[" << this->wrench.body2Force
    //       << "] t1[" << this->wrench.body2Torque
    //       << "] fxp[" << this->wrench.body2Force.Cross(childMomentArm)
    //       << "]\n";

    this->wrench.body2Torque += this->wrench.body2Force.Cross(childMomentArm);

    // rotate resulting body2Force in world frame into link frame
    this->wrench.body2Force = childPose.rot.RotateVectorReverse(
      -this->wrench.body2Force);

    // rotate resulting body2Torque in world frame into link frame
    this->wrench.body2Torque = childPose.rot.RotateVectorReverse(
      -this->wrench.body2Torque);
  }

  // convert torque from about parent CG to joint anchor location
  if (this->parentLink)
  {
    // get child pose, or it's the inertial world if childLink is NULL
    math::Pose childPose;
    if (this->childLink)
      childPose = this->childLink->GetWorldPose();

    math::Pose parentPose = this->parentLink->GetWorldPose();

    // if parent link exists, convert torque from about parent
    // CG to joint anchor location

    // parent cg specified in parent link frame
    math::Pose cgPose = this->parentLink->GetInertial()->GetPose();

    // get parent CG pose in child link frame
    math::Pose parentCGInChildLink =
      math::Pose(cgPose.pos, math::Quaternion()) - (childPose - parentPose);

    // anchor location in parent CG frame
    // this is the moment arm, but it's in parent CG frame, we need
    // to convert it into world frame
    math::Pose anchorInParendCGFrame = this->anchorPose - parentCGInChildLink;

    // paretnCGFrame in world frame
    math::Pose parentCGInWorld = cgPose + parentPose;

    // rotate momeent arms into world frame
    math::Vector3 parentMomentArm = parentCGInWorld.rot.RotateVector(
      (this->anchorPose - parentCGInChildLink).pos);

    // gzerr << "anchor [" << this->anchorPose
    //       << "] pcginc[" << parentCGInChildLink
    //       << "] iarm[" << cgPose
    //       << "] anc2pcg[" << this->anchorPose - parentCGInChildLink
    //       << "] parentMomentArm[" << parentMomentArm
    //       << "] f1[" << this->wrench.body1Force
    //       << "] t1[" << this->wrench.body1Torque
    //       << "] fxp[" << this->wrench.body1Force.Cross(parentMomentArm)
    //       << "]\n";

    this->wrench.body1Torque += this->wrench.body1Force.Cross(parentMomentArm);

    // rotate resulting body1Force in world frame into link frame
    this->wrench.body1Force = parentPose.rot.RotateVectorReverse(
      -this->wrench.body1Force);

    // rotate resulting body1Torque in world frame into link frame
    this->wrench.body1Torque = parentPose.rot.RotateVectorReverse(
      -this->wrench.body1Torque);

    if (!this->childLink)
    {
      // if child link does not exist, use equal and opposite
      this->wrench.body2Force = -this->wrench.body1Force;
      this->wrench.body2Torque = -this->wrench.body1Torque;

      // force/torque are in parent link frame, transform them into
      // child link(world) frame.
      math::Pose parentToWorldTransform = this->parentLink->GetWorldPose();
      this->wrench.body1Force =
        parentToWorldTransform.rot.RotateVector(
        this->wrench.body1Force);
      this->wrench.body1Torque =
        parentToWorldTransform.rot.RotateVector(
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
      math::Pose childToWorldTransform = this->childLink->GetWorldPose();
      this->wrench.body1Force =
        childToWorldTransform.rot.RotateVector(
        this->wrench.body1Force);
      this->wrench.body1Torque =
        childToWorldTransform.rot.RotateVector(
        this->wrench.body1Torque);
    }
  }
  this->wrench = this->wrench - wrenchAppliedWorld;
}

//////////////////////////////////////////////////
JointWrench BulletJoint::GetForceTorque(unsigned int /*_index*/)
{
  return this->wrench;
}

//////////////////////////////////////////////////
void BulletJoint::SetupJointFeedback()
{
  if (this->provideFeedback)
  {
    this->feedback = new btJointFeedback;
    this->feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
    this->feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
    this->feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
    this->feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);

    if (this->constraint)
      this->constraint->setJointFeedback(this->feedback);
    else
    {
      gzerr << "Bullet Joint [" << this->GetName() << "] ID is invalid\n";
      getchar();
    }
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "BulletJoint::SetDamping: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetStiffness(unsigned int _index, double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "BulletJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->GetAngleCount())
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
  if (_index < this->GetAngleCount())
  {
    if (this->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->GetSimTime();
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
  if (_index < this->GetAngleCount())
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
  for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = -fabs(this->dissipationCoefficient[i])
      * this->GetVelocity(i);

    double springForce = this->stiffnessCoefficient[i]
      * (this->springReferencePosition[i] - this->GetAngle(i).Radian());

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
void BulletJoint::SetAnchor(unsigned int /*_index*/,
    const gazebo::math::Vector3 & /*_anchor*/)
{
  // nothing to do here for bullet.
}

//////////////////////////////////////////////////
math::Vector3 BulletJoint::GetAnchor(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Bullet\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 BulletJoint::GetLinkForce(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Bullet\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 BulletJoint::GetLinkTorque(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Bullet\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void BulletJoint::SetAttribute(Attribute, unsigned int /*_index*/,
    double /*_value*/)
{
  gzdbg << "Not implement in Bullet\n";
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
  if (_key == "hi_stop")
  {
    return this->GetHighStop(_index).Radian();
  }
  else if (_key == "lo_stop")
  {
    return this->GetLowStop(_index).Radian();
  }
  gzerr << "GetParam unrecognized parameter ["
        << _key
        << "]"
        << std::endl;
  return 0;
}

//////////////////////////////////////////////////
math::Angle BulletJoint::GetHighStop(unsigned int _index)
{
  return this->GetUpperLimit(_index);
}

//////////////////////////////////////////////////
math::Angle BulletJoint::GetLowStop(unsigned int _index)
{
  return this->GetLowerLimit(_index);
}

//////////////////////////////////////////////////
bool BulletJoint::SetPosition(unsigned int _index, double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}
