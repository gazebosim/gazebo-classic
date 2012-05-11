/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/World.hh"

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletCollision.hh"
#include "physics/bullet/BulletMotionState.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletBody.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletBody::BulletBody(EntityPtr _parent)
    : Body(_parent)
{
  this->rigidBody = NULL;
  this->compoundShape = new btCompoundShape();
  this->motionState = new BulletMotionState(this);
}

//////////////////////////////////////////////////
BulletBody::~BulletBody()
{
}

//////////////////////////////////////////////////
void BulletBody::Load(sdf::ElementPtr _sdf)
{
  this->bulletPhysics = boost::shared_dynamic_cast<BulletPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->bulletPhysics == NULL)
    gzthrow("Not using the bullet physics engine");

  Body::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletBody::Init()
{
  Body::Init();

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
      BulletCollisionPtr collision;
      collision = boost::shared_static_cast<BulletCollision>(*iter);
      btCollisionShape *shape = collision->GetCollisionShape();

      math::Pose relativePose = collision->GetRelativePose();
      relativePose.pos -= cogVec;

      this->compoundShape->addChildShape(
          BulletPhysics::ConvertPose(relativePose), shape);
    }
  }

  this->compoundShape->calculateLocalInertia(btMass, fallInertia);

  // Create a construction info object
  btRigidBody::btRigidBodyConstructionInfo
    rigidBodyCI(btMass, this->motionState, this->compoundShape, fallInertia);

  rigidBodyCI.m_linearDamping = this->GetLinearDamping();
  rigidBodyCI.m_angularDamping = this->GetAngularDamping();

  // Create the new rigid body
  this->rigidBody = new btRigidBody(rigidBodyCI);
  this->rigidBody->setUserPointer(this);
  this->rigidBody->setCollisionFlags(this->rigidBody->getCollisionFlags() |
      btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);


  // Setup motion clamping to prevent objects from moving too fast.
  // this->rigidBody->setCcdMotionThreshold(1);
  // math::Vector3 size = this->GetBoundingBox().GetSize();
  // this->rigidBody->setCcdSweptSphereRadius(size.GetMax()*0.8);

  if (btMass <= 0.0)
    this->rigidBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

  btDynamicsWorld *wd = this->bulletPhysics->GetDynamicsWorld();
  wd->addRigidBody(this->rigidBody);

  // this->rigidBody->setSleepingThresholds(0,0);
}

//////////////////////////////////////////////////
void BulletBody::Fini()
{
  Body::Fini();
}

//////////////////////////////////////////////////
void BulletBody::Update()
{
  Body::Update();
}

//////////////////////////////////////////////////
void BulletBody::SetGravityMode(bool _mode)
{
  if (!this->rigidBody)
    return;

  if (_mode == false)
    this->rigidBody->setGravity(btVector3(0, 0, 0));
    // this->rigidBody->setMassProps(btScalar(0), btmath::Vector3(0, 0, 0));
  else
  {
    math::Vector3 g = this->bulletPhysics->GetGravity();
    this->rigidBody->setGravity(btVector3(g.x, g.y, g.z));
    /*btScalar btMass = this->mass.GetAsDouble();
    btmath::Vector3 fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->rigidBody->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool BulletBody::GetGravityMode()
{
  bool result = false;
  if (this->rigidBody)
  {
    btVector3 g = this->rigidBody->getGravity();
    result = !math::equal(static_cast<double>(g.length()), 0.0);
  }

  return result;
}

//////////////////////////////////////////////////
void BulletBody::SetSelfCollide(bool /*_collide*/)
{
}

//////////////////////////////////////////////////
/*void BulletBody::AttachCollision(Collision *_collision)
{
  Body::AttachCollision(_collision);

  BulletCollision *bcollision = dynamic_cast<BulletCollision*>(_collision);

  if (_collision == NULL)
    gzthrow("requires BulletCollision");

  btTransform trans;
  math::Pose relativePose = _collision->GetRelativePose();
  trans = BulletPhysics::ConvertPose(relativePose);

  bcollision->SetCompoundShapeIndex(this->compoundShape->getNumChildShapes());
  this->compoundShape->addChildShape(trans, bcollision->GetCollisionShape());
}
  */

//////////////////////////////////////////////////
/// changed
void BulletBody::OnPoseChange()
{
  /*
  math::Pose pose = this->GetWorldPose();

  this->motionState->SetWorldPose(pose);
  if (this->rigidBody)
    this->rigidBody->setMotionState(this->motionState);
    */
}

//////////////////////////////////////////////////
void BulletBody::SetEnabled(bool /*_enable*/) const
{
  /*
  if (!this->rigidBody)
    return;

  if (_enable)
    this->rigidBody->activate(true);
  else
    this->rigidBody->setActivationState(WANTS_DEACTIVATION);
    */
}

/////////////////////////////////////////////////////////////////////
/*
  What's going on here?  In ODE the CoM of a body corresponds to the
  origin of the body-fixed coordinate system.  In Gazebo, however, we
  want to have arbitrary body coordinate systems (i.e., CoM may be
  displaced from the body-fixed cs).  To get around this limitation in
  Bullet, we have an extra fudge-factor (comPose), describing the pose of
  the CoM relative to Gazebo's body-fixed cs.  When using low-level
  Bullet functions, one must use apply this factor appropriately.

  The UpdateCoM() function is used to compute this offset, based on
  the mass distribution of attached collisions.  This function also shifts
  the Bullet-pose of the collisions, to keep everything in the same place in
  the Gazebo cs.  Simple, neh?
*/
void BulletBody::UpdateCoM()
{
  // Body::UpdateCoM();
}

//////////////////////////////////////////////////
void BulletBody::SetLinearVel(const math::Vector3 & /*_vel*/)
{
  /*
  if (!this->rigidBody)
    return;

  this->rigidBody->setLinearVelocity(btmath::Vector3(_vel.x, _vel.y, _vel.z));
  */
}

//////////////////////////////////////////////////
math::Vector3 BulletBody::GetWorldLinearVel() const
{
  /*
  if (!this->rigidBody)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec = this->rigidBody->getLinearVelocity();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
void BulletBody::SetAngularVel(const math::Vector3 &_vel)
{
  if (!this->rigidBody)
    return;

  this->rigidBody->setAngularVelocity(btVector3(_vel.x, _vel.y, _vel.z));
}

//////////////////////////////////////////////////
math::Vector3 BulletBody::GetWorldAngularVel() const
{
  if (!this->rigidBody)
    return math::Vector3(0, 0, 0);

  btVector3 btVec = this->rigidBody->getAngularVelocity();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
}

//////////////////////////////////////////////////
void BulletBody::SetForce(const math::Vector3 &/*_force*/)
{
  /*
  if (!this->rigidBody)
    return;

  this->rigidBody->applyCentralForce(
      btmath::Vector3(_force.x, _force.y, _force.z));
      */
}

//////////////////////////////////////////////////
math::Vector3 BulletBody::GetWorldForce() const
{
  /*
  if (!this->rigidBody)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec;

  btVec = this->rigidBody->getTotalForce();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
void BulletBody::SetTorque(const math::Vector3 &_torque)
{
  if (!this->rigidBody)
    return;

  this->rigidBody->applyTorque(btVector3(_torque.x, _torque.y, _torque.z));
}

//////////////////////////////////////////////////
math::Vector3 BulletBody::GetWorldTorque() const
{
  /*
  if (!this->rigidBody)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec;

  btVec = this->rigidBody->getTotalTorque();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
btRigidBody *BulletBody::GetBulletBody() const
{
  return this->rigidBody;
}

//////////////////////////////////////////////////
void BulletBody::SetLinearDamping(double _damping)
{
  if (this->rigidBody)
    this->rigidBody->setDamping((btScalar)_damping,
        (btScalar)this->rigidBody->getAngularDamping());
}

//////////////////////////////////////////////////
void BulletBody::SetAngularDamping(double _damping)
{
  if (this->rigidBody)
    this->rigidBody->setDamping(
        (btScalar)this->rigidBody->getLinearDamping(), (btScalar)_damping);
}

//////////////////////////////////////////////////
/*void BulletBody::SetCollisionRelativePose(BulletCollision *_collision,
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
    // Set the pose of the _collision in Bullet
    this->compoundShape->updateChildTransform(i,
        BulletPhysics::ConvertPose(_newPose));
  }
}*/

/////////////////////////////////////////////////
void BulletBody::AddForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void BulletBody::AddRelativeForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void BulletBody::AddForceAtWorldPosition(const math::Vector3 &_force,
                                         const math::Vector3 &_pos)
{
}

/////////////////////////////////////////////////
void BulletBody::AddForceAtRelativePosition(const math::Vector3 &/*_force*/,
                  const math::Vector3 &/*_relpos*/)
{
}

/////////////////////////////////////////////////
void BulletBody::AddTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void BulletBody::AddRelativeTorque(const math::Vector3 &/*_torque*/)
{
}
