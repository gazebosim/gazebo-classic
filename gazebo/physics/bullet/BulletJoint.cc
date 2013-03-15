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
/* Desc: The base Bullet joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include "common/Exception.hh"
#include "common/Console.hh"

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletJoint::BulletJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->constraint = NULL;
  this->bulletWorld = NULL;
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

  /*if (this->sdf->HasElement("physics") &&
      this->sdf->GetElement("physics")->HasElement("ode"))
  {
    sdf::ElementPtr elem
        = this->sdf->GetElement("physics")->GetElement("bullet");

    if (elem->HasElement("provide_feedback"))
    {
      this->provideFeedback = elem->GetValueBool("provide_feedback");
    }
  }

  if (this->provideFeedback)
  {
    this->feedback = new btJointFeedback;
    this->constraint->setJointFeedback(this->feedback);
  }*/
}

//////////////////////////////////////////////////
void BulletJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr BulletJoint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (this->constraint == NULL)
    gzthrow("Attach bodies to the joint first");

  if (_index == 0 || _index == 1)
  {
    BulletLinkPtr bulletLink1 =
      boost::shared_static_cast<BulletLink>(this->childLink);

    BulletLinkPtr bulletLink2 =
      boost::shared_static_cast<BulletLink>(this->parentLink);

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
JointWrench BulletJoint::GetForceTorque(int _index)
{
  return this->GetForceTorque(static_cast<unsigned int>(_index));
}

//////////////////////////////////////////////////
JointWrench BulletJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench wrench;
  btJointFeedback *fb = this->constraint->getJointFeedback();
  if (fb)
  {
    wrench.body1Force = BulletTypes::ConvertVector3(fb->m_appliedForceBodyB);
    wrench.body2Force = BulletTypes::ConvertVector3(fb->m_appliedForceBodyA);
    wrench.body1Torque = BulletTypes::ConvertVector3(fb->m_appliedTorqueBodyB);
    wrench.body2Torque = BulletTypes::ConvertVector3(fb->m_appliedTorqueBodyA);

/*    gzerr << "num constraints " << this->bulletWorld->getNumConstraints() << std::endl;
    gzerr << "force " << wrench.body1Force << " , " << wrench.body2Force << std::endl;
    gzerr << "torque " << wrench.body1Torque << " , " << wrench.body2Torque << std::endl;

    gzerr << " bt torque " << fb->m_appliedTorqueBodyB.x() << " " << fb->m_appliedTorqueBodyB.y() << " " << fb->m_appliedTorqueBodyB.z() << std::endl;

    gzerr << "anchor " << this->anchorPos << ", " << this->anchorPose << " "<< std::endl;
    gzerr << "global axis " << this->GetGlobalAxis(0) << std::endl;
    gzerr << "angle " << this->GetAngle(0) << std::endl;*/
/*    gzerr << " btbody A " <<
        BulletTypes::ConvertVector3(this->constraint->getRigidBodyA().getGravity())
        << ", " << this->constraint->getRigidBodyA().getInvMass()
        << ", " << BulletTypes::ConvertVector3(this->constraint->getRigidBodyA().getLinearFactor()) << std::endl;

    gzerr << this->childLink->GetInertial()->GetMass() << std::endl;*/

    if (this->childLink)
    {
      math::Pose childPose = this->childLink->GetWorldPose();

      // convert torque from about child CG to joint anchor location
      // cg position specified in child link frame
      math::Vector3 cgPos = this->childLink->GetInertial()->GetPose().pos;

      // moment arm rotated into world frame (given feedback is in world frame)
      math::Vector3 childMomentArm =
        childPose.rot.RotateVector(
        (this->anchorPose - math::Pose(cgPos, math::Quaternion())).pos);

      // gzerr << "anchor [" << anchorPos
      //       << "] iarm[" << this->childLink->GetInertial()->GetPose().pos
      //       << "] childMomentArm[" << childMomentArm
      //       << "] f1[" << wrench.body2Force
      //       << "] t1[" << wrench.body2Torque
      //       << "] fxp[" << wrench.body2Force.Cross(childMomentArm)
      //       << "]\n";

      wrench.body2Torque += wrench.body2Force.Cross(childMomentArm);

      // rotate resulting body1Force in world frame into link frame
      wrench.body2Force = childPose.rot.RotateVectorReverse(
        -wrench.body2Force);

      // rotate resulting body1Torque in world frame into link frame
      wrench.body2Torque = childPose.rot.RotateVectorReverse(
        -wrench.body2Torque);
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
      math::Vector3 cgPos = this->parentLink->GetInertial()->GetPose().pos;

      // rotate momeent arms into world frame
      math::Vector3 parentMomentArm =
        childPose.rot.RotateVector(this->anchorPos - cgPos);

      // gzerr << "anchor [" << anchorPos
      //       << "] iarm[" << cgPos
      //       << "] parentMomentArm[" << parentMomentArm
      //       << "] f1[" << wrench.body1Force
      //       << "] t1[" << wrench.body1Torque
      //       << "] fxp[" << wrench.body1Force.Cross(parentMomentArm)
      //       << "]\n";

//      gzerr << " parent link b4 body1torque " << wrench.body1Torque << std::endl;

      wrench.body1Torque += wrench.body1Force.Cross(parentMomentArm);

//      gzerr << " parent link body1torque " << wrench.body1Torque << std::endl;
//      gzerr << " parentMomentArm " << parentMomentArm << std::endl;
//      gzerr << " cgPos " << cgPos << std::endl;

      // rotate resulting body1Force in world frame into link frame
      wrench.body1Force = parentPose.rot.RotateVectorReverse(
        -wrench.body1Force);

      // rotate resulting body1Torque in world frame into link frame
      wrench.body1Torque = parentPose.rot.RotateVectorReverse(
        -wrench.body1Torque);

      if (!this->childLink)
      {
        // if child link does not exist, use equal and opposite
        wrench.body2Force = -wrench.body1Force;
        wrench.body2Torque = -wrench.body1Torque;
      }
    }
    else
    {
      if (!this->childLink)
      {
        gzerr << "Both parent and child links are invalid, abort.\n";
        return JointWrench();
      }
      else
      {
        // if parentLink does not exist, use equal opposite body1 wrench
        wrench.body1Force = -wrench.body2Force;
        wrench.body1Torque = -wrench.body2Torque;
      }
    }
  }
  else
  {
    // forgot to set provide_feedback?
    gzwarn << "GetForceTorque: forget to set <provide_feedback>?\n";
  }
  return wrench;
}
