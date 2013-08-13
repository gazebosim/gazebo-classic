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

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

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

  // Joint force and torque feedback
  if (_sdf->HasElement("physics") &&
      _sdf->GetElement("physics")->HasElement("bullet"))
  {
    sdf::ElementPtr elem =
      _sdf->GetElement("physics")->GetElement("bullet");

    if (elem->HasElement("provide_feedback"))
    {
      this->SetProvideFeedback(elem->Get<bool>("provide_feedback"));
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
LinkPtr BulletJoint::GetJointLink(int _index) const
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
  // this->forceTorque
  this->forceTorque.body2Force = -BulletTypes::ConvertVector3(
                      this->feedback->m_appliedForceBodyA);
  this->forceTorque.body2Torque = -BulletTypes::ConvertVector3(
                      this->feedback->m_appliedTorqueBodyA);
  this->forceTorque.body1Force = -BulletTypes::ConvertVector3(
                      this->feedback->m_appliedForceBodyB);
  this->forceTorque.body1Torque = -BulletTypes::ConvertVector3(
                      this->feedback->m_appliedTorqueBodyB);
  // gzerr << "   " << this->GetName()
  //       << " : " << this->forceTorque.body1Force
  //       << " : " << this->forceTorque.body1Torque
  //       << " : " << this->forceTorque.body2Force
  //       << " : " << this->forceTorque.body2Torque
  //       << "\n";
}

//////////////////////////////////////////////////
JointWrench BulletJoint::GetForceTorque(int _index)
{
  return this->GetForceTorque(static_cast<unsigned int>(_index));
}

//////////////////////////////////////////////////
JointWrench BulletJoint::GetForceTorque(unsigned int /*_index*/)
{
  return this->forceTorque;
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
