/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Link class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/World.hh"

#include "physics/simbody/simbody_inc.h"
#include "physics/simbody/SimbodyCollision.hh"
#include "physics/simbody/SimbodyMotionState.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodyLink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyLink::SimbodyLink(EntityPtr _parent)
    : Link(_parent)
{
  this->rigidLink = NULL;
  this->compoundShape = new btCompoundShape();
  this->motionState = new SimbodyMotionState(this);
}

//////////////////////////////////////////////////
SimbodyLink::~SimbodyLink()
{
}

//////////////////////////////////////////////////
void SimbodyLink::Load(sdf::ElementPtr _sdf)
{
  this->simbodyPhysics = boost::shared_dynamic_cast<SimbodyPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->simbodyPhysics == NULL)
    gzthrow("Not using the simbody physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyLink::Init()
{
  Link::Init();

  btScalar btMass = this->inertial->GetMass();
  btVector3 fallInertia(0, 0, 0);
  math::Vector3 cogVec = this->inertial->GetCoG();

  // Set the initial pose of the body
  this->motionState->SetWorldPose(this->GetWorldPose());
  this->motionState->SetCoG(cogVec);

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SimbodyCollisionPtr collision;
      collision = boost::shared_static_cast<SimbodyCollision>(*iter);
      btCollisionShape *shape = collision->GetCollisionShape();

      math::Pose relativePose = collision->GetRelativePose();
      relativePose.pos -= cogVec;

      this->compoundShape->addChildShape(
          SimbodyPhysics::ConvertPose(relativePose), shape);
    }
  }

  this->compoundShape->calculateLocalInertia(btMass, fallInertia);

  // Create a construction info object
  btRigidBody::btRigidBodyConstructionInfo
    rigidLinkCI(btMass, this->motionState, this->compoundShape, fallInertia);

  rigidLinkCI.m_linearDamping = this->GetLinearDamping();
  rigidLinkCI.m_angularDamping = this->GetAngularDamping();

  // Create the new rigid body
  this->rigidLink = new btRigidBody(rigidLinkCI);
  this->rigidLink->setUserPointer(this);
  this->rigidLink->setCollisionFlags(this->rigidLink->getCollisionFlags() |
      btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);


  // Setup motion clamping to prevent objects from moving too fast.
  // this->rigidLink->setCcdMotionThreshold(1);
  // math::Vector3 size = this->GetBoundingBox().GetSize();
  // this->rigidLink->setCcdSweptSphereRadius(size.GetMax()*0.8);

  if (btMass <= 0.0)
    this->rigidLink->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

  btDynamicsWorld *wd = this->simbodyPhysics->GetDynamicsWorld();
  wd->addRigidBody(this->rigidLink);

  // this->rigidLink->setSleepingThresholds(0,0);
}

//////////////////////////////////////////////////
void SimbodyLink::Fini()
{
  Link::Fini();
}

//////////////////////////////////////////////////
void SimbodyLink::Update()
{
  Link::Update();
}

//////////////////////////////////////////////////
void SimbodyLink::SetGravityMode(bool _mode)
{
  if (!this->rigidLink)
    return;

  if (_mode == false)
    this->rigidLink->setGravity(btVector3(0, 0, 0));
    // this->rigidLink->setMassProps(btScalar(0), btmath::Vector3(0, 0, 0));
  else
  {
    math::Vector3 g = this->simbodyPhysics->GetGravity();
    this->rigidLink->setGravity(btVector3(g.x, g.y, g.z));
    /*btScalar btMass = this->mass.GetAsDouble();
    btmath::Vector3 fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->rigidLink->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool SimbodyLink::GetGravityMode()
{
  bool result = false;
  if (this->rigidLink)
  {
    btVector3 g = this->rigidLink->getGravity();
    result = !math::equal(static_cast<double>(g.length()), 0.0);
  }

  return result;
}

//////////////////////////////////////////////////
void SimbodyLink::SetSelfCollide(bool /*_collide*/)
{
}

//////////////////////////////////////////////////
/*void SimbodyLink::AttachCollision(Collision *_collision)
{
  Link::AttachCollision(_collision);

  SimbodyCollision *bcollision = dynamic_cast<SimbodyCollision*>(_collision);

  if (_collision == NULL)
    gzthrow("requires SimbodyCollision");

  btTransform trans;
  math::Pose relativePose = _collision->GetRelativePose();
  trans = SimbodyPhysics::ConvertPose(relativePose);

  bcollision->SetCompoundShapeIndex(this->compoundShape->getNumChildShapes());
  this->compoundShape->addChildShape(trans, bcollision->GetCollisionShape());
}
  */

//////////////////////////////////////////////////
/// changed
void SimbodyLink::OnPoseChange()
{
  /*
  math::Pose pose = this->GetWorldPose();

  this->motionState->SetWorldPose(pose);
  if (this->rigidLink)
    this->rigidLink->setMotionState(this->motionState);
    */
}

//////////////////////////////////////////////////
void SimbodyLink::SetEnabled(bool /*_enable*/) const
{
  /*
  if (!this->rigidLink)
    return;

  if (_enable)
    this->rigidLink->activate(true);
  else
    this->rigidLink->setActivationState(WANTS_DEACTIVATION);
    */
}

/////////////////////////////////////////////////////////////////////
/*
  What's going on here?  In ODE the CoM of a body corresponds to the
  origin of the body-fixed coordinate system.  In Gazebo, however, we
  want to have arbitrary body coordinate systems (i.e., CoM may be
  displaced from the body-fixed cs).  To get around this limitation in
  Simbody, we have an extra fudge-factor (comPose), describing the pose of
  the CoM relative to Gazebo's body-fixed cs.  When using low-level
  Simbody functions, one must use apply this factor appropriately.

  The UpdateCoM() function is used to compute this offset, based on
  the mass distribution of attached collisions.  This function also shifts
  the Simbody-pose of the collisions, to keep everything in the same place in
  the Gazebo cs.  Simple, neh?
*/
void SimbodyLink::UpdateCoM()
{
  // Link::UpdateCoM();
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearVel(const math::Vector3 & /*_vel*/)
{
  /*
  if (!this->rigidLink)
    return;

  this->rigidLink->setLinearVelocity(btmath::Vector3(_vel.x, _vel.y, _vel.z));
  */
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldLinearVel() const
{
  /*
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec = this->rigidLink->getLinearVelocity();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularVel(const math::Vector3 &_vel)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->setAngularVelocity(btVector3(_vel.x, _vel.y, _vel.z));
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldAngularVel() const
{
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btVector3 btVec = this->rigidLink->getAngularVelocity();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
}

//////////////////////////////////////////////////
void SimbodyLink::SetForce(const math::Vector3 &/*_force*/)
{
  /*
  if (!this->rigidLink)
    return;

  this->rigidLink->applyCentralForce(
      btmath::Vector3(_force.x, _force.y, _force.z));
      */
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldForce() const
{
  /*
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec;

  btVec = this->rigidLink->getTotalForce();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyLink::SetTorque(const math::Vector3 &_torque)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->applyTorque(btVector3(_torque.x, _torque.y, _torque.z));
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldTorque() const
{
  /*
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec;

  btVec = this->rigidLink->getTotalTorque();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
btRigidBody *SimbodyLink::GetSimbodyLink() const
{
  return this->rigidLink;
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearDamping(double _damping)
{
  if (this->rigidLink)
    this->rigidLink->setDamping((btScalar)_damping,
        (btScalar)this->rigidLink->getAngularDamping());
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularDamping(double _damping)
{
  if (this->rigidLink)
    this->rigidLink->setDamping(
        (btScalar)this->rigidLink->getLinearDamping(), (btScalar)_damping);
}

//////////////////////////////////////////////////
/*void SimbodyLink::SetCollisionRelativePose(SimbodyCollision *_collision,
    const math::Pose &_newPose)
{
  std::map<std::string, Collision*>::iterator iter;
  unsigned int i;

  for (iter = this->collisions.begin(), i = 0; iter != this->collisions.end();
       ++iter, ++i)
  {
    if (iter->second == _collision)
      break;
  }

  if (i < this->collisions.size())
  {
    // Set the pose of the _collision in Simbody
    this->compoundShape->updateChildTransform(i,
        SimbodyPhysics::ConvertPose(_newPose));
  }
}*/

/////////////////////////////////////////////////
void SimbodyLink::AddForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddRelativeForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddForceAtWorldPosition(const math::Vector3 &/*_force*/,
                                         const math::Vector3 &/*_pos*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddForceAtRelativePosition(const math::Vector3 &/*_force*/,
                  const math::Vector3 &/*_relpos*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddRelativeTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::SetAutoDisable(bool /*_disable*/)
{
}
