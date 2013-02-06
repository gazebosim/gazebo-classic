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
/* Desc: Link class
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
#include "physics/bullet/BulletLink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletLink::BulletLink(EntityPtr _parent)
    : Link(_parent)
{
  this->rigidLink = NULL;
  this->compoundShape = new btCompoundShape();
}

//////////////////////////////////////////////////
BulletLink::~BulletLink()
{
}

//////////////////////////////////////////////////
void BulletLink::Load(sdf::ElementPtr _sdf)
{
  this->bulletPhysics = boost::shared_dynamic_cast<BulletPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->bulletPhysics == NULL)
    gzthrow("Not using the bullet physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletLink::Init()
{
  Link::Init();

  btScalar mass = this->inertial->GetMass();
  // The bullet dynamics solver checks for zero mass to identify static and
  // kinematic bodies.
  if (this->IsStatic() || this->GetKinematic())
  {
    mass = 0;
    this->inertial->SetInertiaMatrix(0, 0, 0, 0, 0, 0);
  }
  btVector3 fallInertia(0, 0, 0);
  math::Vector3 cogVec = this->inertial->GetCoG();

  // Set the initial pose of the body
  this->motionState.reset(new BulletMotionState(
    boost::shared_dynamic_cast<Link>(shared_from_this())));

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
          BulletTypes::ConvertPose(relativePose), shape);
    }
  }

  // this->compoundShape->calculateLocalInertia(mass, fallInertia);
  fallInertia = BulletTypes::ConvertVector3(
    this->inertial->GetPrincipalMoments());
  // TODO: inertia products not currently used
  this->inertial->SetInertiaMatrix(fallInertia.x(), fallInertia.y(),
                                   fallInertia.z(), 0, 0, 0);

  // Create a construction info object
  btRigidBody::btRigidBodyConstructionInfo
    rigidLinkCI(mass, this->motionState.get(), this->compoundShape,
    fallInertia);

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

  if (mass <= 0.0)
    this->rigidLink->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

  btDynamicsWorld *wd = this->bulletPhysics->GetDynamicsWorld();
  wd->addRigidBody(this->rigidLink);

  // this->rigidLink->setSleepingThresholds(0,0);
}

//////////////////////////////////////////////////
void BulletLink::Fini()
{
  Link::Fini();
}

//////////////////////////////////////////////////
void BulletLink::Update()
{
  Link::Update();
}

//////////////////////////////////////////////////
void BulletLink::SetGravityMode(bool _mode)
{
  if (!this->rigidLink)
    return;

  if (_mode == false)
    this->rigidLink->setGravity(btVector3(0, 0, 0));
    // this->rigidLink->setMassProps(btScalar(0), btmath::Vector3(0, 0, 0));
  else
  {
    math::Vector3 g = this->bulletPhysics->GetGravity();
    this->rigidLink->setGravity(btVector3(g.x, g.y, g.z));
    /*btScalar btMass = this->mass.GetAsDouble();
    btmath::Vector3 fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->rigidLink->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool BulletLink::GetGravityMode() const
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
void BulletLink::SetSelfCollide(bool /*_collide*/)
{
}

//////////////////////////////////////////////////
/*void BulletLink::AttachCollision(Collision *_collision)
{
  Link::AttachCollision(_collision);

  BulletCollision *bcollision = dynamic_cast<BulletCollision*>(_collision);

  if (_collision == NULL)
    gzthrow("requires BulletCollision");

  btTransform trans;
  math::Pose relativePose = _collision->GetRelativePose();
  trans = BulletTypes::ConvertPose(relativePose);

  bcollision->SetCompoundShapeIndex(this->compoundShape->getNumChildShapes());
  this->compoundShape->addChildShape(trans, bcollision->GetCollisionShape());
}
  */

//////////////////////////////////////////////////
/// Adapted from ODELink::OnPoseChange
void BulletLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->rigidLink)
    return;

  // this->SetEnabled(true);

  const math::Pose myPose = this->GetWorldCoGPose();

  this->rigidLink->setCenterOfMassTransform(
    BulletTypes::ConvertPose(myPose));
}

//////////////////////////////////////////////////
bool BulletLink::GetEnabled() const
{
  // This function and its counterpart BulletLink::SetEnabled
  // don't do anything yet.
  return true;
}

//////////////////////////////////////////////////
void BulletLink::SetEnabled(bool /*_enable*/) const
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

//////////////////////////////////////////////////
void BulletLink::SetLinearVel(const math::Vector3 &_vel)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->setLinearVelocity(BulletTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
math::Vector3 BulletLink::GetWorldLinearVel() const
{
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btVector3 vel = this->rigidLink->getLinearVelocity();

  return math::Vector3(vel.x(), vel.y(), vel.z());
}

//////////////////////////////////////////////////
void BulletLink::SetAngularVel(const math::Vector3 &_vel)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->setAngularVelocity(BulletTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
math::Vector3 BulletLink::GetWorldAngularVel() const
{
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btVector3 vel = this->rigidLink->getAngularVelocity();

  return math::Vector3(vel.x(), vel.y(), vel.z());
}

//////////////////////////////////////////////////
void BulletLink::SetForce(const math::Vector3 &/*_force*/)
{
  /*
  if (!this->rigidLink)
    return;

  this->rigidLink->applyCentralForce(
      btmath::Vector3(_force.x, _force.y, _force.z));
      */
}

//////////////////////////////////////////////////
math::Vector3 BulletLink::GetWorldForce() const
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
void BulletLink::SetTorque(const math::Vector3 &_torque)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->applyTorque(btVector3(_torque.x, _torque.y, _torque.z));
}

//////////////////////////////////////////////////
math::Vector3 BulletLink::GetWorldTorque() const
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
btRigidBody *BulletLink::GetBulletLink() const
{
  return this->rigidLink;
}

//////////////////////////////////////////////////
void BulletLink::SetLinearDamping(double _damping)
{
  if (this->rigidLink)
    this->rigidLink->setDamping((btScalar)_damping,
        (btScalar)this->rigidLink->getAngularDamping());
}

//////////////////////////////////////////////////
void BulletLink::SetAngularDamping(double _damping)
{
  if (this->rigidLink)
    this->rigidLink->setDamping(
        (btScalar)this->rigidLink->getLinearDamping(), (btScalar)_damping);
}

//////////////////////////////////////////////////
/*void BulletLink::SetCollisionRelativePose(BulletCollision *_collision,
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
        BulletTypes::ConvertPose(_newPose));
  }
}*/

/////////////////////////////////////////////////
void BulletLink::AddForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void BulletLink::AddRelativeForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void BulletLink::AddForceAtWorldPosition(const math::Vector3 &/*_force*/,
                                         const math::Vector3 &/*_pos*/)
{
}

/////////////////////////////////////////////////
void BulletLink::AddForceAtRelativePosition(const math::Vector3 &/*_force*/,
                  const math::Vector3 &/*_relpos*/)
{
}

/////////////////////////////////////////////////
void BulletLink::AddTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void BulletLink::AddRelativeTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void BulletLink::SetAutoDisable(bool /*_disable*/)
{
}
