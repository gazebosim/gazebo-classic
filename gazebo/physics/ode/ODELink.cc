/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <math.h>
#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldPrivate.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"

#include "gazebo/physics/ode/ODEPrivateLink.hh"
#include "gazebo/physics/ode/ODELink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODELink::ODELink(EntityPtr _parent)
: Link(*new ODELinkPrivate, _parent),
  dataPtr(std::static_pointer_cast<ODELinkPrivate>(this->linkDPtr)
{
  this->dataPtr->linkId = NULL;
}

//////////////////////////////////////////////////
ODELink::~ODELink()
{
  if (this->dataPtr->linkId)
    dBodyDestroy(this->dataPtr->linkId);
  this->dataPtr->linkId = NULL;
}

//////////////////////////////////////////////////
void ODELink::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->odePhysics = boost::dynamic_pointer_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->dataPtr->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void ODELink::Init()
{
  if (!this->IsStatic())
  {
    this->dataPtr->linkId =
      dBodyCreate(this->dataPtr->odePhysics->GetWorldId());

    dBodySetData(this->dataPtr->linkId, this);

    // Only use auto disable if no joints and no sensors are present
    if (this->Model()->GetAutoDisable() &&
        this->Model()->GetJointCount() == 0 &&
        this->SensorCount() == 0)
    {
      dBodySetAutoDisableDefaults(this->dataPtr->linkId);
      dBodySetAutoDisableFlag(this->dataPtr->linkId, 1);
    }
    else
    {
      dBodySetAutoDisableFlag(this->dataPtr->linkId, 0);
    }
  }

  GZ_ASSERT(this->dataPtr->sdf != NULL,
      "Unable to initialize link, SDF is NULL");
  this->SetKinematic(this->dataPtr->sdf->Get<bool>("kinematic"));
  this->SetGravityMode(this->dataPtr->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->LinearDamping());
  this->SetAngularDamping(this->AngularDamping());

  Link::Init();

  if (this->dataPtr->linkId)
  {
    GZ_ASSERT(this->dataPtr->inertial != NULL, "Inertial pointer is NULL");
    ignition::math::Vector3d cogVec = this->dataPtr->inertial->GetCoG().Ign();
    for (auto const &child : this->dataPtr->children)
    {
      if (child->HasType(Base::COLLISION))
      {
        ODECollisionPtr g = boost::static_pointer_cast<ODECollision>(child);
        if (g->IsPlaceable() && g->CollisionId())
        {
          dGeomSetBody(g->CollisionId(), this->dataPtr->linkId);
        }
      }
    }
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to place collision bodies"
          << " in ODELink::Init" << std::endl;
  }

  // Update the Collision Offsets, Surface, and Center of Mass.
  this->UpdateCollisionOffsets();
  this->UpdateSurface();
  // Only update the Center of Mass if object is dynamic
  if (!this->Kinematic())
    this->UpdateMass();

  if (this->dataPtr->linkId)
  {
    dBodySetMovedCallback(this->dataPtr->linkId, MoveCallback);
    dBodySetDisabledCallback(this->dataPtr->linkId, DisabledCallback);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to set callbacks in ODELink::Init"
          << std::endl;
  }
}

//////////////////////////////////////////////////
void ODELink::DisabledCallback(dBodyID /*_id*/)
{
}

//////////////////////////////////////////////////
void ODELink::MoveCallback(dBodyID _id)
{
  const dReal *p;
  const dReal *r;
  ODELink *self = static_cast<ODELink*>(dBodyGetData(_id));
  // self->poseMutex->lock();

  p = dBodyGetPosition(_id);
  r = dBodyGetQuaternion(_id);

  self->dirtyPose.pos.Set(p[0], p[1], p[2]);
  self->dirtyPose.rot.Set(r[0], r[1], r[2], r[3]);

  // subtracting cog location from ode pose
  GZ_ASSERT(self->inertial != NULL, "Inertial pointer is NULL");
  ignition::math::Vector3d cog = self->dataPtr->dirtyPose.Rot().RotateVector(
      self->dataPtr->inertial->GetCoG().Ign());

  self->dataPtr->dirtyPose.Pos() -= cog;

  // Tell the world that our pose has changed.
  self->dataPtr->world->_AddDirty(self);

  // self->poseMutex->unlock();

  // get force and applied to this body
  if (_id)
  {
    const dReal *dforce = dBodyGetForce(_id);
    self->force.Set(dforce[0], dforce[1], dforce[2]);

    const dReal *dtorque = dBodyGetTorque(_id);
    self->torque.Set(dtorque[0], dtorque[1], dtorque[2]);
  }
}

//////////////////////////////////////////////////
void ODELink::Fini()
{
  Link::Fini();
  if (this->dataPtr->linkId)
    dBodyDestroy(this->dataPtr->linkId);
  this->dataPtr->linkId = NULL;

  this->dataPtr->odePhysics.reset();
}

//////////////////////////////////////////////////
void ODELink::SetGravityMode(bool _mode)
{
  this->dataPtr->sdf->GetElement("gravity")->Set(_mode);
  if (this->dataPtr->linkId)
  {
    dBodySetGravityMode(this->dataPtr->linkId, _mode ? 1: 0);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetGravityMode" << std::endl;
  }
}

//////////////////////////////////////////////////
bool ODELink::GetGravityMode() const
{
  int mode = 0;
  if (this->dataPtr->linkId)
  {
    mode = dBodyGetGravityMode(this->dataPtr->linkId);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetGravityMode returns default of "
          << mode << std::endl;
  }

  return mode;
}

//////////////////////////////////////////////////
void ODELink::SetSelfCollide(bool _collide)
{
  this->dataPtr->sdf->GetElement("self_collide")->Set(_collide);
  if (_collide)
    this->spaceId = dSimpleSpaceCreate(this->dataPtr->odePhysics->GetSpaceId());
}

//////////////////////////////////////////////////
void ODELink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->dataPtr->linkId)
  {
    if (!this->IsStatic() && this->dataPtr->initialized)
      gzlog << "ODE body for link [" << this->ScopedName() << "]"
            << " does not exist, unable to respond to OnPoseChange"
            << std::endl;
    return;
  }

  this->SetEnabled(true);

  const ignition::math::Pose3d myPose = this->WorldPose();

  GZ_ASSERT(this->dataPtr->inertial != NULL, "Inertial pointer is NULL");
  math::Vector3 cog = myPose.Rot().RotateVector(
      this->dataPtr->inertial->GetCoG().Ign());

  // adding cog location for ode pose
  dBodySetPosition(this->dataPtr->linkId,
      myPose.Pos.X() + cog.X(),
      myPose.Pos.Y() + cog.Y(),
      myPose.Pos.Z() + cog.Z());

  dQuaternion q;
  q[0] = myPose.Rot().W();
  q[1] = myPose.Rot().X();
  q[2] = myPose.Rot().Y();
  q[3] = myPose.Rot().Z();

  // Set the rotation of the ODE link
  dBodySetQuaternion(this->dataPtr->linkId, q);
}

//////////////////////////////////////////////////
dBodyID ODELink::GetODEId() const
{
  return this->ODEId();
}

//////////////////////////////////////////////////
dBodyID ODELink::ODEId() const
{
  return this->dataPtr->linkId;
}

//////////////////////////////////////////////////
void ODELink::SetEnabled(bool _enable) const
{
  if (!this->dataPtr->linkId)
  {
    if (!this->IsStatic() && this->dataPtr->initialized)
      gzlog << "ODE body for link [" << this->ScopedName() << "]"
            << " does not exist, unable to SetEnabled" << std::endl;
    return;
  }

  if (_enable)
    dBodyEnable(this->dataPtr->linkId);
  else
    dBodyDisable(this->dataPtr->linkId);
}

/////////////////////////////////////////////////////////////////////
bool ODELink::GetEnabled() const
{
  bool result = true;

  if (this->dataPtr->linkId)
    result = dBodyIsEnabled(this->dataPtr->linkId);
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetEnabled returns default of "
          << result << std::endl;
  }

  return result;
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateCollisionOffsets()
{
  if (this->dataPtr->linkId)
  {
    GZ_ASSERT(this->dataPtr->inertial != NULL, "Inertial pointer is NULL");
    math::Vector3 cogVec = this->dataPtr->inertial->GetCoG();
    for (auto const &child : this->dataPtr->children)
    {
      if (child->HasType(Base::COLLISION))
      {
        ODECollisionPtr g = boost::static_pointer_cast<ODECollision>(child);
        if (g->IsPlaceable() && g->GetCollisionId())
        {
          // update pose immediately
          math::Pose localPose = g->GetRelativePose();
          localPose.pos -= cogVec;

          dQuaternion q;
          q[0] = localPose.rot.w;
          q[1] = localPose.rot.x;
          q[2] = localPose.rot.y;
          q[3] = localPose.rot.z;

          // Set the pose of the encapsulated collision; this is always relative
          // to the CoM
          dGeomSetOffsetPosition(g->GetCollisionId(), localPose.pos.x,
              localPose.pos.y, localPose.pos.z);
          dGeomSetOffsetQuaternion(g->GetCollisionId(), q);
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateSurface()
{
  if (!this->dataPtr->linkId)
  {
    return;
  }

  for (auto const &child : this->dataPtr->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      ODECollisionPtr g = boost::static_pointer_cast<ODECollision>(child);
      if (g->IsPlaceable() && g->GetCollisionId())
      {
        // Set max_vel and min_depth
        boost::any value;
        if (g->GetODESurface()->maxVel < 0 && this->GetWorld()->
            GetPhysicsEngine()->GetParam("contact_max_correcting_vel", value))
        {
          try
          {
            g->GetODESurface()->maxVel = boost::any_cast<double>(value);
          }
          catch(boost::bad_any_cast &_e)
          {
            gzerr << "Failed boost::any_cast in ODELink.cc: " << _e.what();
          }
        }
        // Set surface properties max_vel and min_depth
        dBodySetMaxVel(this->dataPtr->linkId, g->GetODESurface()->maxVel);
        dBodySetMinDepth(this->dataPtr->linkId, g->GetODESurface()->minDepth);
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateMass()
{
  if (!this->dataPtr->linkId)
  {
    if (!this->IsStatic() && this->dataPtr->initialized)
      gzlog << "ODE body for link [" << this->ScopedName() << "]"
            << " does not exist, unable to UpdateMass" << std::endl;
    return;
  }

  dMass odeMass;
  dMassSetZero(&odeMass);

  // The CoG must always be (0, 0, 0)
  math::Vector3 cog(0, 0, 0);

  GZ_ASSERT(this->dataPtr->inertial != NULL, "Inertial pointer is NULL");
  // give ODE un-rotated inertia
  math::Matrix3 moi = this->dataPtr->inertial->GetMOI(
    math::Pose(this->dataPtr->inertial->GetCoG(), math::Quaternion()));
  math::Vector3 principals(moi[0][0], moi[1][1], moi[2][2]);
  math::Vector3 products(moi[0][1], moi[0][2], moi[1][2]);

  dMassSetParameters(&odeMass, this->dataPtr->inertial->GetMass(),
      cog.x, cog.y, cog.z,
      principals.x, principals.y, principals.z,
      products.x, products.y, products.z);

  if (this->dataPtr->inertial->GetMass() > 0)
    dBodySetMass(this->dataPtr->linkId, &odeMass);
  else
    gzthrow("Setting custom link " + this->ScopedName() + "mass to zero!");

  // In case the center of mass changed:
  this->UpdateCollisionOffsets();
  this->OnPoseChange();
}

//////////////////////////////////////////////////
void ODELink::SetLinearVel(const math::Vector3 &_vel)
{
  if (this->dataPtr->linkId)
  {
    dBodySetLinearVel(this->dataPtr->linkId, _vel.x, _vel.y, _vel.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldLinearVel(const math::Vector3 &_offset) const
{
  math::Vector3 vel;

  if (this->dataPtr->linkId)
  {
    dVector3 dvel;
    GZ_ASSERT(this->dataPtr->inertial != NULL, "Inertial pointer is NULL");
    math::Vector3 offsetFromCoG = _offset - this->dataPtr->inertial->GetCoG();
    dBodyGetRelPointVel(this->dataPtr->linkId, offsetFromCoG.x, offsetFromCoG.y,
        offsetFromCoG.z, dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetWorldLinearVel returns default of "
          << vel << std::endl;
  }
  return vel;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldLinearVel(const math::Vector3 &_offset,
                                         const math::Quaternion &_q) const
{
  math::Vector3 vel;

  if (this->dataPtr->linkId)
  {
    dVector3 dvel;
    math::Pose wPose = this->WorldPose();
    GZ_ASSERT(this->dataPtr->inertial != NULL, "Inertial pointer is NULL");
    math::Vector3 offsetFromCoG =
        wPose.rot.RotateVectorReverse(_q * _offset)
        - this->dataPtr->inertial->GetCoG();
    dBodyGetRelPointVel(this->dataPtr->linkId, offsetFromCoG.x, offsetFromCoG.y,
        offsetFromCoG.z, dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetWorldLinearVel returns default of "
          << vel << std::endl;
  }

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldCoGLinearVel() const
{
  math::Vector3 vel;

  if (this->dataPtr->linkId)
  {
    const dReal *dvel;
    dvel = dBodyGetLinearVel(this->dataPtr->linkId);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetWorldCoGLinearVel returns default of "
          << vel << std::endl;
  }

  return vel;
}

//////////////////////////////////////////////////
void ODELink::SetAngularVel(const math::Vector3 &_vel)
{
  if (this->dataPtr->linkId)
  {
    dBodySetAngularVel(this->dataPtr->linkId, _vel.x, _vel.y, _vel.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldAngularVel() const
{
  math::Vector3 vel;

  if (this->dataPtr->linkId)
  {
    const dReal *dvel;

    dvel = dBodyGetAngularVel(this->dataPtr->linkId);

    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetWorldAngularVel returns default of "
          << vel << std::endl;
  }

  return vel;
}

//////////////////////////////////////////////////
void ODELink::SetForce(const math::Vector3 &_force)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodySetForce(this->dataPtr->linkId, _force.x, _force.y, _force.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetForce" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetTorque(const math::Vector3 &_torque)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodySetTorque(this->dataPtr->linkId, _torque.x, _torque.y, _torque.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetTorque" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::AddForce(const math::Vector3 &_force)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForce(this->dataPtr->linkId, _force.x, _force.y, _force.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddForce" << std::endl;
}

/////////////////////////////////////////////////
void ODELink::AddRelativeForce(const math::Vector3 &_force)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodyAddRelForce(this->dataPtr->linkId, _force.x, _force.y, _force.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddRelativeForce" << std::endl;
}

/////////////////////////////////////////////////
void ODELink::AddForceAtRelativePosition(const math::Vector3 &_force,
                               const math::Vector3 &_relpos)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtRelPos(this->dataPtr->linkId, _force.x, _force.y, _force.z,
                          _relpos.x, _relpos.y, _relpos.z);
  }
  else if (!this->IsStatic())
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddForceAtRelativePosition"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void ODELink::AddForceAtWorldPosition(const math::Vector3 &_force,
                                      const math::Vector3 &_pos)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtPos(this->dataPtr->linkId, _force.x, _force.y, _force.z,
                          _pos.x, _pos.y, _pos.z);
  }
  else if (!this->IsStatic())
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddForceAtWorldPosition"
          << std::endl;
  }
}

//////////////////////////////////////////////////
void ODELink::AddLinkForce(const math::Vector3 &_force,
    const math::Vector3 &_offset)
{
  if (this->dataPtr->linkId)
  {
    // Force vector represents a direction only, so it should be rotated but
    // not translated
    math::Vector3 forceWorld = this->WorldPose().Rot().RotateVector(_force);
    // Does this need to be rotated?
    math::Vector3 offsetCoG = _offset - this->dataPtr->inertial->GetCoG();

    this->SetEnabled(true);
    dBodyAddForceAtRelPos(this->dataPtr->linkId,
        forceWorld.x, forceWorld.y, forceWorld.z,
        offsetCoG.x, offsetCoG.y, offsetCoG.z);
  }
  else if (!this->IsStatic())
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddLinkForce"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void ODELink::AddTorque(const math::Vector3 &_torque)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodyAddTorque(this->dataPtr->linkId, _torque.x, _torque.y, _torque.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddTorque" << std::endl;
}

/////////////////////////////////////////////////
void ODELink::AddRelativeTorque(const math::Vector3 &_torque)
{
  if (this->dataPtr->linkId)
  {
    this->SetEnabled(true);
    dBodyAddRelTorque(this->dataPtr->linkId, _torque.x, _torque.y, _torque.z);
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to AddRelativeTorque" << std::endl;
}

/////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldForce() const
{
  return this->force;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldTorque() const
{
  return this->torque;
}

//////////////////////////////////////////////////
dSpaceID ODELink::GetSpaceId() const
{
  return this->spaceId;
}

//////////////////////////////////////////////////
void ODELink::SetSpaceId(dSpaceID _spaceid)
{
  this->spaceId = _spaceid;
}

//////////////////////////////////////////////////
void ODELink::SetLinearDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetLinearDamping(this->GetODEId(), _damping);
  else if (!this->IsStatic() && this->dataPtr->initialized)
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetLinearDamping" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetAngularDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetAngularDamping(this->GetODEId(), _damping);
  else if (!this->IsStatic() && this->dataPtr->initialized)
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetAngularDamping" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetKinematic(const bool &_state)
{
  this->dataPtr->sdf->GetElement("kinematic")->Set(_state);
  if (this->dataPtr->linkId)
  {
    if (_state && !dBodyIsKinematic(this->dataPtr->linkId))
      dBodySetKinematic(this->dataPtr->linkId);
    else if (dBodyIsKinematic(this->dataPtr->linkId))
      dBodySetDynamic(this->dataPtr->linkId);
  }
  else if (!this->IsStatic() && this->dataPtr->initialized)
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetKinematic" << std::endl;
}

//////////////////////////////////////////////////
bool ODELink::GetKinematic() const
{
  bool result = false;

  if (this->dataPtr->linkId)
    result = dBodyIsKinematic(this->dataPtr->linkId);
  else if (!this->IsStatic() && this->dataPtr->initialized)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, GetKinematic returns default of "
          << result << std::endl;
  }

  return result;
}

//////////////////////////////////////////////////
void ODELink::SetAutoDisable(bool _disable)
{
  if (this->GetModel()->GetJointCount() == 0 && this->dataPtr->linkId)
  {
    dBodySetAutoDisableFlag(this->dataPtr->linkId, _disable);
  }
  else if (!this->dataPtr->linkId)
  {
    gzlog << "ODE body for link [" << this->ScopedName() << "]"
          << " does not exist, unable to SetAutoDisable" << std::endl;
  }
  else
  {
    gzlog << "ODE model has joints, unable to SetAutoDisable" << std::endl;
  }
}

//////////////////////////////////////////////////
void ODELink::SetLinkStatic(bool /*_static*/)
{
  gzlog << "To be implemented\n";
}
