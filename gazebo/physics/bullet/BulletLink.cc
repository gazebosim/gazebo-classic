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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"

#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletMotionState.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"

#include "gazebo/physics/bullet/BulletLinkPrivate.hh"
#include "gazebo/physics/bullet/BulletLink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletLink::BulletLink(EntityPtr _parent)
: Link(*new BulletLinkPrivate,_parent),
  bulletLinkDPtr(static_cast<BulletLinkPrivate*>(this->linkDPtr))
{
  this->bulletLinkDPtr->rigidLink = NULL;
  this->bulletLinkDPtr->compoundShape = NULL;
}

//////////////////////////////////////////////////
BulletLink::~BulletLink()
{
  delete this->bulletLinkDPtr->compoundShape;
}

//////////////////////////////////////////////////
void BulletLink::Load(sdf::ElementPtr _sdf)
{
  this->bulletLinkDPtr->bulletPhysics =
    std::dynamic_pointer_cast<BulletPhysics>(
        this->World()->Physics());

  if (this->bulletLinkDPtr->bulletPhysics == NULL)
  {
    gzerr << "Not using the bullet physics engine\n";
    return;
  }

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletLink::Init()
{
  // Set the initial pose of the body
  this->bulletLinkDPtr->motionState.reset(new BulletMotionState(
    std::dynamic_pointer_cast<Link>(shared_from_this())));

  Link::Init();

  GZ_ASSERT(this->bulletLinkDPtr->sdf != NULL,
      "Unable to initialize link, SDF is NULL");

  this->SetKinematic(this->bulletLinkDPtr->sdf->Get<bool>("kinematic"));

  btScalar mass = this->bulletLinkDPtr->inertial.Mass();
  // The bullet dynamics solver checks for zero mass to identify static and
  // kinematic bodies.
  if (this->IsStatic() || this->Kinematic())
  {
    mass = 0;
    this->bulletLinkDPtr->inertial.SetInertiaMatrix(0, 0, 0, 0, 0, 0);
  }

  btVector3 fallInertia(0, 0, 0);
  ignition::math::Vector3d cogVec = this->bulletLinkDPtr->inertial.CoG();

  /// \todo FIXME:  Friction Parameters
  /// Currently, gazebo uses btCompoundShape to store multiple
  /// <collision> shapes in bullet.  Each child shape could have a
  /// different mu1 and mu2.  This is not ideal as friction is set
  /// per BulletLink::rigidLink (btRigidBody : btCollisionObject).
  /// Right now, the friction coefficients for the last BulletCollision
  /// processed in this link below is stored in hackMu1, hackMu2.
  /// The average is stored in this->bulletLinkDPtr->rigidLink.
  /// Final friction coefficient is applied in ContactCallback
  /// by taking the lower of the 2 colliding rigidLink's.
  double hackMu1 = 0;
  double hackMu2 = 0;

  for (Base_V::iterator iter = this->bulletLinkDPtr->children.begin();
       iter != this->bulletLinkDPtr->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      BulletCollisionPtr collision;
      collision = std::static_pointer_cast<BulletCollision>(*iter);
      btCollisionShape *shape = collision->CollisionShape();

      SurfaceParamsPtr surface = collision->Surface();
      GZ_ASSERT(surface, "Surface pointer for is invalid");
      FrictionPyramidPtr friction = surface->FrictionPyramid();
      GZ_ASSERT(friction, "Friction pointer is invalid");

      hackMu1 = friction->MuPrimary();
      hackMu2 = friction->MuSecondary();
      // gzerr << "link[" << this->Name()
      //       << "] mu[" << hackMu1
      //       << "] mu2[" << hackMu2 << "]\n";

      ignition::math::Pose3d relativePose = collision->RelativePose();
      relativePose.Pos() -= cogVec;

      if (!this->bulletLinkDPtr->compoundShape)
        this->bulletLinkDPtr->compoundShape = new btCompoundShape();
      dynamic_cast<btCompoundShape *>(
          this->bulletLinkDPtr->compoundShape)->addChildShape(
            BulletTypes::ConvertPose(relativePose), shape);
    }
  }

  // if there are no collisions in the link then use an empty shape
  if (!this->bulletLinkDPtr->compoundShape)
    this->bulletLinkDPtr->compoundShape = new btEmptyShape();

  // this->compoundShape->calculateLocalInertia(mass, fallInertia);
  fallInertia = BulletTypes::ConvertVector3(
    this->bulletLinkDPtr->inertial.PrincipalMoments());
  // TODO: inertia products not currently used
  this->bulletLinkDPtr->inertial.SetInertiaMatrix(
      fallInertia.x(), fallInertia.y(), fallInertia.z(), 0, 0, 0);

  // Create a construction info object
  btRigidBody::btRigidBodyConstructionInfo
    rigidLinkCI(mass, this->bulletLinkDPtr->motionState.get(),
        this->bulletLinkDPtr->compoundShape, fallInertia);

  rigidLinkCI.m_linearDamping = this->LinearDamping();
  rigidLinkCI.m_angularDamping = this->AngularDamping();

  // Create the new rigid body
  this->bulletLinkDPtr->rigidLink = new btRigidBody(rigidLinkCI);
  this->bulletLinkDPtr->rigidLink->setUserPointer(this);
  this->bulletLinkDPtr->rigidLink->setCollisionFlags(
      this->bulletLinkDPtr->rigidLink->getCollisionFlags() |
      btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
  this->bulletLinkDPtr->rigidLink->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);

  /// \TODO: get friction from collision object
  this->bulletLinkDPtr->rigidLink->setAnisotropicFriction(btVector3(1, 1, 1),
    btCollisionObject::CF_ANISOTROPIC_FRICTION);

  // Hack
  this->bulletLinkDPtr->rigidLink->setFriction(0.5*(hackMu1 + hackMu2));

  // Setup motion clamping to prevent objects from moving too fast.
  // this->bulletLinkDPtr->rigidLink->setCcdMotionThreshold(1);
  // ignition::math::Vector3d size = this->GetBoundingBox().GetSize();
  // this->bulletLinkDPtr->rigidLink->setCcdSweptSphereRadius(size.GetMax()*0.8);

  if (mass <= 0.0)
  {
    this->bulletLinkDPtr->rigidLink->setCollisionFlags(
        btCollisionObject::CF_KINEMATIC_OBJECT);
  }

  btDynamicsWorld *bulletWorld =
    this->bulletLinkDPtr->bulletPhysics->DynamicsWorld();
  GZ_ASSERT(bulletWorld != NULL, "Bullet dynamics world is NULL");

  // bullet supports setting bits to a rigid body but not individual
  // shapes/collisions so find the first child collision and set rigid body to
  // use its category and collision bits.
  unsigned int categortyBits = GZ_ALL_COLLIDE;
  unsigned int collideBits = GZ_ALL_COLLIDE;
  BulletCollisionPtr collision;
  for (Base_V::iterator iter = this->bulletLinkDPtr->children.begin();
         iter != this->bulletLinkDPtr->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      collision = std::static_pointer_cast<BulletCollision>(*iter);
      categortyBits = collision->CategoryBits();
      collideBits = collision->CollideBits();
      break;
    }
  }
  bulletWorld->addRigidBody(this->bulletLinkDPtr->rigidLink,
      categortyBits, collideBits);

  // Only use auto disable if no joints and no sensors are present
  this->bulletLinkDPtr->rigidLink->setActivationState(DISABLE_DEACTIVATION);
  if (this->ParentModel()->AutoDisable() &&
      this->ParentModel()->JointCount() == 0 &&
      this->SensorCount() == 0)
  {
    this->bulletLinkDPtr->rigidLink->setActivationState(ACTIVE_TAG);
    this->bulletLinkDPtr->rigidLink->setSleepingThresholds(0.1, 0.1);
    this->bulletLinkDPtr->rigidLink->setDeactivationTime(1.0);
  }

  this->SetGravityMode(this->bulletLinkDPtr->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->LinearDamping());
  this->SetAngularDamping(this->AngularDamping());
}

//////////////////////////////////////////////////
void BulletLink::Fini()
{
  Link::Fini();
  btDynamicsWorld *bulletWorld =
    this->bulletLinkDPtr->bulletPhysics->DynamicsWorld();

  GZ_ASSERT(bulletWorld != NULL, "Bullet dynamics world is NULL");
  bulletWorld->removeRigidBody(this->bulletLinkDPtr->rigidLink);
}

/////////////////////////////////////////////////////////////////////
void BulletLink::UpdateMass()
{
  if (this->bulletLinkDPtr->rigidLink)
  {
    this->bulletLinkDPtr->rigidLink->setMassProps(
        this->bulletLinkDPtr->inertial.Mass(),
        BulletTypes::ConvertVector3(
          this->bulletLinkDPtr->inertial.PrincipalMoments()));
  }
}

//////////////////////////////////////////////////
void BulletLink::SetGravityMode(const bool _mode)
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to SetGravityMode" << std::endl;
    return;
  }

  if (_mode == false)
  {
    this->bulletLinkDPtr->rigidLink->setGravity(btVector3(0, 0, 0));
    // this->bulletLinkDPtr->rigidLink->setMassProps(btScalar(0),
    // btignition::math::Vector3d(0, 0, 0));
  }
  else
  {
    ignition::math::Vector3d g =
      this->bulletLinkDPtr->bulletPhysics->Gravity();
    this->bulletLinkDPtr->rigidLink->setGravity(btVector3(g.X(), g.Y(), g.Z()));
    /*btScalar btMass = this->mass.GetAsDouble();
    btignition::math::Vector3d fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->bulletLinkDPtr->rigidLink->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool BulletLink::GravityMode() const
{
  bool result = false;
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, GravityMode returns "
          << result << " by default." << std::endl;
    return result;
  }
  btVector3 g = this->bulletLinkDPtr->rigidLink->getGravity();
  result = !ignition::math::equal(static_cast<double>(g.length()), 0.0);

  return result;
}

//////////////////////////////////////////////////
void BulletLink::SetSelfCollide(const bool _collide)
{
  this->bulletLinkDPtr->sdf->GetElement("self_collide")->Set(_collide);
}

//////////////////////////////////////////////////
/*void BulletLink::AttachCollision(Collision *_collision)
{
  Link::AttachCollision(_collision);

  BulletCollision *bcollision = dynamic_cast<BulletCollision*>(_collision);

  if (_collision == NULL)
    gzthrow("requires BulletCollision");

  btTransform trans;
  ignition::math::Pose3d relativePose = _collision->GetRelativePose();
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

  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to respond to OnPoseChange"
          << std::endl;
    return;
  }

  // this->SetEnabled(true);

  const ignition::math::Pose3d myPose = this->WorldCoGPose();

  this->bulletLinkDPtr->rigidLink->setCenterOfMassTransform(
    BulletTypes::ConvertPose(myPose));
}

//////////////////////////////////////////////////
bool BulletLink::Enabled() const
{
  // This function and its counterpart BulletLink::SetEnabled
  // don't do anything yet.
  return true;
}

//////////////////////////////////////////////////
void BulletLink::SetEnabled(const bool /*_enable*/) const
{
  /*
  if (!this->bulletLinkDPtr->rigidLink)
    return;

  if (_enable)
    this->bulletLinkDPtr->rigidLink->activate(true);
  else
    this->bulletLinkDPtr->rigidLink->setActivationState(WANTS_DEACTIVATION);
    */
}

//////////////////////////////////////////////////
void BulletLink::SetLinearVel(const ignition::math::Vector3d &_vel)
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
    return;
  }

  this->bulletLinkDPtr->rigidLink->setLinearVelocity(BulletTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldCoGLinearVel() const
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  btVector3 vel = this->bulletLinkDPtr->rigidLink->getLinearVelocity();

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldLinearVel(
    const ignition::math::Vector3d &_offset) const
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  ignition::math::Pose3d wPose = this->WorldPose();
  ignition::math::Vector3d offsetFromCoG = wPose.Rot() *
    (_offset - this->bulletLinkDPtr->inertial.CoG());
  btVector3 vel = this->bulletLinkDPtr->rigidLink->getVelocityInLocalPoint(
      BulletTypes::ConvertVector3(offsetFromCoG));

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldLinearVel(
    const ignition::math::Vector3d &_offset,
    const ignition::math::Quaterniond &_q) const
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  ignition::math::Pose3d wPose = this->WorldPose();
  ignition::math::Vector3d offsetFromCoG = _q * _offset
        - wPose.Rot() * this->bulletLinkDPtr->inertial.CoG();
  btVector3 vel = this->bulletLinkDPtr->rigidLink->getVelocityInLocalPoint(
      BulletTypes::ConvertVector3(offsetFromCoG));

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
void BulletLink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  this->bulletLinkDPtr->rigidLink->setAngularVelocity(BulletTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldAngularVel() const
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, GetWorldAngularVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  btVector3 vel = this->bulletLinkDPtr->rigidLink->getAngularVelocity();

  return BulletTypes::ConvertVector3Ign(vel);
}

//////////////////////////////////////////////////
void BulletLink::SetForce(const ignition::math::Vector3d &_force)
{
  if (!this->bulletLinkDPtr->rigidLink)
    return;

  this->bulletLinkDPtr->rigidLink->applyCentralForce(
    btVector3(_force.X(), _force.Y(), _force.Z()));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldForce() const
{
  if (!this->bulletLinkDPtr->rigidLink)
    return ignition::math::Vector3d(0, 0, 0);

  btVector3 btVec;

  btVec = this->bulletLinkDPtr->rigidLink->getTotalForce();

  return ignition::math::Vector3d(btVec.x(), btVec.y(), btVec.z());
}

//////////////////////////////////////////////////
void BulletLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  this->bulletLinkDPtr->rigidLink->applyTorque(BulletTypes::ConvertVector3(_torque));
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletLink::WorldTorque() const
{
  /*
  if (!this->bulletLinkDPtr->rigidLink)
    return ignition::math::Vector3d(0, 0, 0);

  btignition::math::Vector3d btVec;

  btVec = this->bulletLinkDPtr->rigidLink->getTotalTorque();

  return ignition::math::Vector3d(btVec.x(), btVec.y(), btVec.z());
  */
  return ignition::math::Vector3d();
}

//////////////////////////////////////////////////
btRigidBody *BulletLink::BtLink() const
{
  return this->bulletLinkDPtr->rigidLink;
}

//////////////////////////////////////////////////
void BulletLink::ClearCollisionCache()
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to ClearCollisionCache" << std::endl;
    return;
  }

  btDynamicsWorld *bulletWorld =
    this->bulletLinkDPtr->bulletPhysics->DynamicsWorld();
  GZ_ASSERT(bulletWorld != NULL, "Bullet dynamics world is NULL");

  bulletWorld->updateSingleAabb(this->bulletLinkDPtr->rigidLink);
  bulletWorld->getBroadphase()->getOverlappingPairCache()->
    cleanProxyFromPairs(this->bulletLinkDPtr->rigidLink->getBroadphaseHandle(),
        bulletWorld->getDispatcher());
}

//////////////////////////////////////////////////
void BulletLink::SetLinearDamping(const double _damping)
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to SetLinearDamping"
          << std::endl;
    return;
  }
  this->bulletLinkDPtr->rigidLink->setDamping((btScalar)_damping,
      (btScalar)this->bulletLinkDPtr->rigidLink->getAngularDamping());
}

//////////////////////////////////////////////////
void BulletLink::SetAngularDamping(const double _damping)
{
  if (!this->bulletLinkDPtr->rigidLink)
  {
    gzlog << "Bullet rigid body for link [" << this->Name() << "]"
          << " does not exist, unable to SetAngularDamping"
          << std::endl;
    return;
  }

  this->bulletLinkDPtr->rigidLink->setDamping(
      (btScalar)this->bulletLinkDPtr->rigidLink->getLinearDamping(),
      (btScalar)_damping);
}

//////////////////////////////////////////////////
// void BulletLink::SetCollisionRelativePose(BulletCollision *_collision,
//     const ignition::math::Pose3d &_newPose)
// {
//   std::map<std::string, Collision*>::iterator iter;
//   unsigned int i;
//
//   for (iter = this->collisions.begin(), i = 0;
//        iter != this->collisions.end(); ++iter, ++i)
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
void BulletLink::SetAutoDisable(const bool /*_disable*/)
{
  gzlog << "BulletLink::SetAutoDisable not yet implemented." << std::endl;
}

//////////////////////////////////////////////////
void BulletLink::SetLinkStatic(const bool /*_static*/)
{
  gzlog << "To be implemented\n";
}

//////////////////////////////////////////////////
BulletMotionStatePtr BulletLink::MotionState() const
{
  return this->bulletLinkDPtr->motionState;
}
