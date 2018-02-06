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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"

#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletMotionState.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletLink::BulletLink(EntityPtr _parent)
    : Link(_parent)
{
  this->rigidLink = nullptr;
  this->compoundShape = nullptr;

  this->bulletPhysics = boost::dynamic_pointer_cast<BulletPhysics>(
      this->GetWorld()->Physics());
  if (this->bulletPhysics == nullptr)
    gzerr << "Not using the bullet physics engine\n";
}

//////////////////////////////////////////////////
BulletLink::~BulletLink()
{
  this->Fini();
}

//////////////////////////////////////////////////
void BulletLink::Load(sdf::ElementPtr _sdf)
{
  if (this->bulletPhysics)
    Link::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletLink::Init()
{
  if (!this->bulletPhysics)
    return;

  // Set the initial pose of the body
  this->motionState.reset(new BulletMotionState(
    boost::dynamic_pointer_cast<Link>(shared_from_this())));

  Link::Init();

  GZ_ASSERT(this->sdf != nullptr, "Unable to initialize link, SDF is null");
  this->SetKinematic(this->sdf->Get<bool>("kinematic"));

  GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
  // The bullet dynamics solver checks for zero mass to identify static and
  // kinematic bodies.
  if (this->IsStatic() || this->GetKinematic())
  {
    this->inertial->SetMass(0);
    this->inertial->SetInertiaMatrix(0, 0, 0, 0, 0, 0);
  }
  else
  {
    // Diagonalize inertia matrix and add inertial pose rotation
    auto inertiald = this->inertial->Ign();
    auto m = inertiald.MassMatrix();
    auto Idiag = m.PrincipalMoments();
    auto inertialPose = inertiald.Pose();
    inertialPose.Rot() *= m.PrincipalAxesOffset();

    this->inertial->SetInertiaMatrix(Idiag[0], Idiag[1], Idiag[2], 0, 0, 0);
    this->inertial->SetCoG(inertialPose);
  }

  /// \todo FIXME:  Friction Parameters
  /// Currently, gazebo uses btCompoundShape to store multiple
  /// <collision> shapes in bullet.  Each child shape could have a
  /// different mu1 and mu2.  This is not ideal as friction is set
  /// per BulletLink::rigidLink (btRigidBody : btCollisionObject).
  /// Right now, the friction coefficients for the last BulletCollision
  /// processed in this link below is stored in hackMu1, hackMu2.
  /// The average is stored in this->rigidLink.
  /// Final friction coefficient is applied in ContactCallback
  /// by taking the lower of the 2 colliding rigidLink's.
  double hackMu1 = 0;
  double hackMu2 = 0;

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      BulletCollisionPtr collision;
      collision = boost::static_pointer_cast<BulletCollision>(*iter);
      btCollisionShape *shape = collision->GetCollisionShape();

      SurfaceParamsPtr surface = collision->GetSurface();
      GZ_ASSERT(surface, "Surface pointer is invalid");
      FrictionPyramidPtr friction = surface->FrictionPyramid();
      GZ_ASSERT(friction, "Friction pointer is invalid");

      hackMu1 = friction->MuPrimary();
      hackMu2 = friction->MuSecondary();

      if (!shape)
      {
        gzerr << "No collision shape initialized. Skipping.\n";
        continue;
      }

      auto relativePose = collision->RelativePose()
          - this->inertial->Pose();
      if (!this->compoundShape)
        this->compoundShape = new btCompoundShape();
      dynamic_cast<btCompoundShape *>(this->compoundShape)->addChildShape(
          BulletTypes::ConvertPose(relativePose), shape);
    }
  }

  // if there are no collisions in the link then use an empty shape
  if (!this->compoundShape)
    this->compoundShape = new btEmptyShape();

  // Create a construction info object
  btRigidBody::btRigidBodyConstructionInfo
      rigidLinkCI(this->inertial->Mass(), this->motionState.get(),
      this->compoundShape, BulletTypes::ConvertVector3(
      this->inertial->PrincipalMoments()));

  rigidLinkCI.m_linearDamping = this->GetLinearDamping();
  rigidLinkCI.m_angularDamping = this->GetAngularDamping();

  // Create the new rigid body
  this->rigidLink = new btRigidBody(rigidLinkCI);
  this->rigidLink->setUserPointer(this);
  this->rigidLink->setCollisionFlags(this->rigidLink->getCollisionFlags() |
      btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
  this->rigidLink->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);

  /// \TODO: get friction from collision object
  this->rigidLink->setAnisotropicFriction(btVector3(1, 1, 1),
    btCollisionObject::CF_ANISOTROPIC_FRICTION);
  this->rigidLink->setFriction(0.5*(hackMu1 + hackMu2));  // Hack

  // Setup motion clamping to prevent objects from moving too fast.
  // this->rigidLink->setCcdMotionThreshold(1);
  // auto size = this->BoundingBox().Size();
  // this->rigidLink->setCcdSweptSphereRadius(size.GetMax()*0.8);

  if (this->inertial->Mass() <= 0.0)
    this->rigidLink->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

  btDynamicsWorld *bulletWorld = this->bulletPhysics->GetDynamicsWorld();
  GZ_ASSERT(bulletWorld != nullptr, "Bullet dynamics world is null");

  // bullet supports setting bits to a rigid body but not individual
  // shapes/collisions so find the first child collision and set rigid body to
  // use its category and collision bits.
  unsigned int collideBits = GZ_ALL_COLLIDE;
  BulletCollisionPtr collision;
  for (Base_V::iterator iter = this->children.begin();
         iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      collision = boost::static_pointer_cast<BulletCollision>(*iter);
      collideBits = collision->GetCollideBits();
      break;
    }
  }
  bulletWorld->addRigidBody(this->rigidLink, collideBits, collideBits);

  // Only use auto disable if no joints and no sensors are present
  this->rigidLink->setActivationState(DISABLE_DEACTIVATION);
  if (this->GetModel()->GetAutoDisable() &&
      this->GetModel()->GetJointCount() == 0 &&
      this->GetSensorCount() == 0)
  {
    this->rigidLink->setActivationState(ACTIVE_TAG);
    this->rigidLink->setSleepingThresholds(0.1, 0.1);
    this->rigidLink->setDeactivationTime(1.0);
  }

  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->GetLinearDamping());
  this->SetAngularDamping(this->GetAngularDamping());
}

//////////////////////////////////////////////////
void BulletLink::Fini()
{
  if (this->bulletPhysics && this->rigidLink)
  {
    btDynamicsWorld *bulletWorld = this->bulletPhysics->GetDynamicsWorld();
    if (bulletWorld)
      bulletWorld->removeRigidBody(this->rigidLink);

    delete this->rigidLink;
  }
  this->bulletPhysics.reset();
  this->rigidLink = nullptr;

  this->motionState.reset();

  if (this->compoundShape)
    delete this->compoundShape;
  this->compoundShape = nullptr;

  Link::Fini();
}

/////////////////////////////////////////////////////////////////////
void BulletLink::UpdateMass()
{
  if (this->rigidLink && this->inertial)
  {
    if (this->inertial->ProductsOfInertia() != ignition::math::Vector3d::Zero)
    {
      gzwarn << "UpdateMass is ignoring off-diagonal inertia terms.\n";
    }
    this->rigidLink->setMassProps(this->inertial->Mass(),
        BulletTypes::ConvertVector3(this->inertial->PrincipalMoments()));
  }
}

//////////////////////////////////////////////////
void BulletLink::SetGravityMode(bool _mode)
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetGravityMode" << std::endl;
    return;
  }

  if (_mode == false)
    this->rigidLink->setGravity(btVector3(0, 0, 0));
    // this->rigidLink->setMassProps(btScalar(0), btVector3(0, 0, 0));
  else
  {
    auto g = this->world->Gravity();
    this->rigidLink->setGravity(btVector3(g.X(), g.Y(), g.Z()));
    /*btScalar btMass = this->mass.GetAsDouble();
    btVector3 fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->rigidLink->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool BulletLink::GetGravityMode() const
{
  bool result = false;
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetGravityMode returns "
          << result << " by default." << std::endl;
    return result;
  }
  btVector3 g = this->rigidLink->getGravity();
  result = !ignition::math::equal(static_cast<double>(g.length()), 0.0);

  return result;
}

//////////////////////////////////////////////////
void BulletLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);
}

//////////////////////////////////////////////////
// void BulletLink::AttachCollision(Collision *_collision)
// {
//   Link::AttachCollision(_collision);
//
//   BulletCollision *bcollision = dynamic_cast<BulletCollision*>(_collision);
//
//   if (_collision == nullptr)
//     gzthrow("requires BulletCollision");
//
//   btTransform trans;
//   ignition::math::Pose3d relativePose = _collision->RelativePose();
//   trans = BulletTypes::ConvertPose(relativePose);
//
//   bcollision->SetCompoundShapeIndex(
//   this->compoundShape->getNumChildShapes());
//   this->compoundShape->addChildShape(trans, bcollision->GetCollisionShape());
// }

//////////////////////////////////////////////////
/// Adapted from ODELink::OnPoseChange
void BulletLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to respond to OnPoseChange"
          << std::endl;
    return;
  }

  // this->SetEnabled(true);

  const ignition::math::Pose3d myPose = this->WorldInertialPose();

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
void BulletLink::SetLinearVel(const ignition::math::Vector3d &_vel)
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
    return;
  }

  this->rigidLink->setLinearVelocity(BulletTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldCoGLinearVel() const
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, WorldLinearVel returns "
          << ignition::math::Vector3d::Zero << " by default." << std::endl;
    return ignition::math::Vector3d::Zero;
  }

  btVector3 vel = this->rigidLink->getLinearVelocity();

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldLinearVel(
    const ignition::math::Vector3d &_offset) const
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, WorldLinearVel returns "
          << ignition::math::Vector3d::Zero << " by default." << std::endl;
    return ignition::math::Vector3d::Zero;
  }

  ignition::math::Pose3d wPose = this->WorldPose();
  GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
  ignition::math::Vector3d offsetFromCoG = wPose.Rot()*
    (_offset - this->inertial->CoG());
  btVector3 vel = this->rigidLink->getVelocityInLocalPoint(
      BulletTypes::ConvertVector3(offsetFromCoG));

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldLinearVel(
    const ignition::math::Vector3d &_offset,
    const ignition::math::Quaterniond &_q) const
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, WorldLinearVel returns "
          << ignition::math::Vector3d::Zero << " by default." << std::endl;
    return ignition::math::Vector3d::Zero;
  }

  ignition::math::Pose3d wPose = this->WorldPose();
  GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
  ignition::math::Vector3d offsetFromCoG = _q*_offset
        - wPose.Rot()*this->inertial->CoG();
  btVector3 vel = this->rigidLink->getVelocityInLocalPoint(
      BulletTypes::ConvertVector3(offsetFromCoG));

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
void BulletLink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  this->rigidLink->setAngularVelocity(BulletTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldAngularVel() const
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, WorldAngularVel returns "
          << ignition::math::Vector3d::Zero << " by default." << std::endl;
    return ignition::math::Vector3d::Zero;
  }

  btVector3 vel = this->rigidLink->getAngularVelocity();

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
void BulletLink::SetForce(const ignition::math::Vector3d &_force)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->applyCentralForce(
    btVector3(_force.X(), _force.Y(), _force.Z()));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldForce() const
{
  if (!this->rigidLink)
    return ignition::math::Vector3d(0, 0, 0);

  btVector3 btVec;

  btVec = this->rigidLink->getTotalForce();

  return ignition::math::Vector3d(btVec.x(), btVec.y(), btVec.z());
}

//////////////////////////////////////////////////
void BulletLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  this->rigidLink->applyTorque(BulletTypes::ConvertVector3(_torque));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldTorque() const
{
  // if (!this->rigidLink)
  //   return ignition::math::Vector3d(0, 0, 0);
  // btVector3 btVec;
  // btVec = this->rigidLink->getTotalTorque();
  // return ignition::math::Vector3d(btVec.x(), btVec.y(), btVec.z());

  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
btRigidBody *BulletLink::GetBulletLink() const
{
  return this->rigidLink;
}


//////////////////////////////////////////////////
void BulletLink::RemoveAndAddBody() const
{
  GZ_ASSERT(nullptr != this->rigidLink, "Must add body to world first");

  btDynamicsWorld *bulletWorld = this->bulletPhysics->GetDynamicsWorld();
  bulletWorld->removeRigidBody(this->rigidLink);

  unsigned int collideBits = GZ_ALL_COLLIDE;
  for (auto iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      auto collision = boost::static_pointer_cast<BulletCollision>(*iter);
      collideBits = collision->GetCollideBits();
      break;
    }
  }

  bulletWorld->addRigidBody(this->rigidLink, collideBits, collideBits);
}

//////////////////////////////////////////////////
void BulletLink::ClearCollisionCache()
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to ClearCollisionCache" << std::endl;
    return;
  }

  btDynamicsWorld *bulletWorld = this->bulletPhysics->GetDynamicsWorld();
  GZ_ASSERT(bulletWorld != nullptr, "Bullet dynamics world is null");

  bulletWorld->updateSingleAabb(this->rigidLink);
  bulletWorld->getBroadphase()->getOverlappingPairCache()->
      cleanProxyFromPairs(this->rigidLink->getBroadphaseHandle(),
      bulletWorld->getDispatcher());
}

//////////////////////////////////////////////////
void BulletLink::SetLinearDamping(double _damping)
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearDamping"
          << std::endl;
    return;
  }
  this->rigidLink->setDamping((btScalar)_damping,
      (btScalar)this->rigidLink->getAngularDamping());
}

//////////////////////////////////////////////////
void BulletLink::SetAngularDamping(double _damping)
{
  if (!this->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularDamping"
          << std::endl;
    return;
  }
  this->rigidLink->setDamping(
      (btScalar)this->rigidLink->getLinearDamping(), (btScalar)_damping);
}

//////////////////////////////////////////////////
// void BulletLink::SetCollisionRelativePose(BulletCollision *_collision,
//     const ignition::math::Pose3d &_newPose)
// {
//   std::map<std::string, Collision*>::iterator iter;
//   unsigned int i;
//
//   for (iter = this->collisions.begin(), i = 0;
//        iter != this->collisions.end();
//        ++iter, ++i)
//   {
//     if (iter->second == _collision)
//       break;
//   }
//
//   if (i < this->collisions.size())
//   {
//     // Set the pose of the _collision in Bullet
//     this->compoundShape->updateChildTransform(i,
//         BulletTypes::ConvertPose(_newPose));
//   }
// }

/////////////////////////////////////////////////
void BulletLink::AddForce(const ignition::math::Vector3d &/*_force*/)
{
  gzlog << "BulletLink::AddForce not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void BulletLink::AddRelativeForce(const ignition::math::Vector3d &/*_force*/)
{
  gzlog << "BulletLink::AddRelativeForce not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void BulletLink::AddForceAtWorldPosition(
    const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_pos*/)
{
  gzlog << "BulletLink::AddForceAtWorldPosition not yet implemented."
        << std::endl;
}

/////////////////////////////////////////////////
void BulletLink::AddForceAtRelativePosition(
    const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_relpos*/)
{
  gzlog << "BulletLink::AddForceAtRelativePosition not yet implemented."
        << std::endl;
}

//////////////////////////////////////////////////
void BulletLink::AddLinkForce(const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_offset*/)
{
  gzlog << "BulletLink::AddLinkForce not yet implemented (#1476)."
        << std::endl;
}

/////////////////////////////////////////////////
void BulletLink::AddTorque(const ignition::math::Vector3d &/*_torque*/)
{
  gzlog << "BulletLink::AddTorque not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void BulletLink::AddRelativeTorque(const ignition::math::Vector3d &/*_torque*/)
{
  gzlog << "BulletLink::AddRelativeTorque not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void BulletLink::SetAutoDisable(bool /*_disable*/)
{
  gzlog << "BulletLink::SetAutoDisable not yet implemented." << std::endl;
}

//////////////////////////////////////////////////
void BulletLink::SetLinkStatic(bool /*_static*/)
{
  gzlog << "To be implemented\n";
}
