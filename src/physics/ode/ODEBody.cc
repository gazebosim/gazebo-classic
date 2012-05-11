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

#include <math.h>
#include <sstream>

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/Collision.hh"
#include "physics/World.hh"
#include "physics/Model.hh"
#include "physics/ode/ODECollision.hh"
#include "physics/SurfaceParams.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODEBody.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEBody::ODEBody(EntityPtr _parent)
    : Body(_parent)
{
  this->bodyId = NULL;
}

//////////////////////////////////////////////////
ODEBody::~ODEBody()
{
  if (this->bodyId)
    dBodyDestroy(this->bodyId);
  this->bodyId = NULL;
}

//////////////////////////////////////////////////
void ODEBody::Load(sdf::ElementPtr _sdf)
{
  this->odePhysics = boost::shared_dynamic_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  Body::Load(_sdf);
}

//////////////////////////////////////////////////
void ODEBody::Init()
{
  if (!this->IsStatic())
  {
    this->bodyId = dBodyCreate(this->odePhysics->GetWorldId());
    dBodySetData(this->bodyId, this);

    // Only use auto disable if no joints are present
    if (this->GetModel()->GetJointCount() == 0)
    {
      dBodySetAutoDisableDefaults(this->bodyId);
      dBodySetAutoDisableFlag(this->bodyId, 0);
    }
  }

  Body::Init();

  if (this->bodyId)
  {
    math::Vector3 cogVec = this->inertial->GetCoG();
    Base_V::iterator iter;
    for (iter = this->children.begin(); iter != this->children.end(); ++iter)
    {
      if ((*iter)->HasType(Base::COLLISION))
      {
        ODECollisionPtr g = boost::shared_static_cast<ODECollision>(*iter);
        if (g->IsPlaceable() && g->GetCollisionId())
        {
          dGeomSetBody(g->GetCollisionId(), this->bodyId);

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

          // Set max_vel and min_depth
          if (g->GetSurface()->maxVel < 0)
          {
            g->GetSurface()->maxVel =
             this->GetWorld()->GetPhysicsEngine()->GetContactMaxCorrectingVel();
          }
          dBodySetMaxVel(this->bodyId, g->GetSurface()->maxVel);
          dBodySetMinDepth(this->bodyId, g->GetSurface()->minDepth);
        }
      }
    }
  }

  // Update the Center of Mass.
  this->UpdateMass();

  if (this->bodyId)
  {
    dBodySetMovedCallback(this->bodyId, MoveCallback);
    dBodySetDisabledCallback(this->bodyId, DisabledCallback);
  }
}

//////////////////////////////////////////////////
void ODEBody::DisabledCallback(dBodyID /*_id*/)
{
  printf("Disabled\n");
}

//////////////////////////////////////////////////
void ODEBody::MoveCallback(dBodyID _id)
{
  const dReal *p;
  const dReal *r;
  ODEBody *self = static_cast<ODEBody*>(dBodyGetData(_id));
  // self->poseMutex->lock();

  p = dBodyGetPosition(_id);
  r = dBodyGetQuaternion(_id);

  self->dirtyPose.pos.Set(p[0], p[1], p[2]);
  self->dirtyPose.rot.Set(r[0], r[1], r[2], r[3]);

  // subtracting cog location from ode pose
  math::Vector3 cog = self->dirtyPose.rot.RotateVector(
      self->inertial->GetCoG());

  self->dirtyPose.pos -= cog;

  // TODO: this is an ugly line of code. It's like this for speed.
  self->world->dirtyPoses.push_back(self);

  // self->poseMutex->unlock();
}

//////////////////////////////////////////////////
void ODEBody::Fini()
{
  Body::Fini();
  if (this->bodyId)
    dBodyDestroy(this->bodyId);
  this->bodyId = NULL;

  this->odePhysics.reset();
}

//////////////////////////////////////////////////
void ODEBody::Update()
{
  Body::Update();
}

//////////////////////////////////////////////////
void ODEBody::SetGravityMode(bool _mode)
{
  this->sdf->GetAttribute("gravity")->Set(_mode);
  if (this->bodyId)
  {
    dBodySetGravityMode(this->bodyId, _mode ? 1: 0);
  }
}

//////////////////////////////////////////////////
bool ODEBody::GetGravityMode()
{
  int mode = 0;
  if (this->bodyId)
  {
    mode = dBodyGetGravityMode(this->bodyId);
  }

  return mode;
}

//////////////////////////////////////////////////
void ODEBody::SetSelfCollide(bool _collide)
{
  this->sdf->GetAttribute("self_collide")->Set(_collide);
  if (_collide && !this->spaceId)
    this->spaceId = dSimpleSpaceCreate(this->odePhysics->GetSpaceId());
}

//////////////////////////////////////////////////
void ODEBody::OnPoseChange()
{
  Body::OnPoseChange();

  if (!this->bodyId)
    return;

  this->SetEnabled(true);

  const math::Pose myPose = this->GetWorldPose();

  math::Vector3 cog = myPose.rot.RotateVector(this->inertial->GetCoG());

  // adding cog location for ode pose
  dBodySetPosition(this->bodyId,
      myPose.pos.x + cog.x,
      myPose.pos.y + cog.y,
      myPose.pos.z + cog.z);

  dQuaternion q;
  q[0] = myPose.rot.w;
  q[1] = myPose.rot.x;
  q[2] = myPose.rot.y;
  q[3] = myPose.rot.z;

  // Set the rotation of the ODE body
  dBodySetQuaternion(this->bodyId, q);
}

//////////////////////////////////////////////////
dBodyID ODEBody::GetODEId() const
{
  return this->bodyId;
}


//////////////////////////////////////////////////
void ODEBody::SetEnabled(bool _enable) const
{
  if (!this->bodyId)
    return;

  if (_enable)
    dBodyEnable(this->bodyId);
  else
    dBodyDisable(this->bodyId);
}

/////////////////////////////////////////////////////////////////////
bool ODEBody::GetEnabled() const
{
  bool result = true;

  if (this->bodyId)
    result = dBodyIsEnabled(this->bodyId);

  return result;
}

/////////////////////////////////////////////////////////////////////
void ODEBody::UpdateSurface()
{
  Base_V::iterator iter;
  Base_V::iterator iter_end = this->children.end();
  for (iter = this->children.begin(); iter != iter_end; ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      ODECollisionPtr g = boost::shared_static_cast<ODECollision>(*iter);
      if (g->IsPlaceable() && g->GetCollisionId())
      {
        // Set surface properties max_vel and min_depth
        dBodySetMaxVel(this->bodyId, g->GetSurface()->maxVel);
        dBodySetMinDepth(this->bodyId, g->GetSurface()->minDepth);
      }
    }
  }
}
/////////////////////////////////////////////////////////////////////
void ODEBody::UpdateMass()
{
  if (!this->bodyId)
    return;

  dMass odeMass;
  dMassSetZero(&odeMass);

  // The CoG must always be (0, 0, 0)
  math::Vector3 cog(0, 0, 0);

  math::Vector3 principals = this->inertial->GetPrincipalMoments();
  math::Vector3 products = this->inertial->GetProductsofInertia();

  dMassSetParameters(&odeMass, this->inertial->GetMass(),
      cog.x, cog.y, cog.z,
      principals.x, principals.y, principals.z,
      products.x, products.y, products.z);

  if (this->inertial->GetMass() > 0)
    dBodySetMass(this->bodyId, &odeMass);
  else
    gzthrow("Setting custom body " + this->GetName() + "mass to zero!");
}

//////////////////////////////////////////////////
void ODEBody::SetLinearVel(const math::Vector3 &_vel)
{
  if (this->bodyId)
  {
    dBodySetLinearVel(this->bodyId, _vel.x, _vel.y, _vel.z);
  }
}

//////////////////////////////////////////////////
math::Vector3 ODEBody::GetWorldLinearVel() const
{
  math::Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;
    dvel = dBodyGetLinearVel(this->bodyId);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
void ODEBody::SetAngularVel(const math::Vector3 &_vel)
{
  if (this->bodyId)
  {
    dBodySetAngularVel(this->bodyId, _vel.x, _vel.y, _vel.z);
  }
}

//////////////////////////////////////////////////
math::Vector3 ODEBody::GetWorldAngularVel() const
{
  math::Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    dvel = dBodyGetAngularVel(this->bodyId);

    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
void ODEBody::SetForce(const math::Vector3 &_force)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodySetForce(this->bodyId, _force.x, _force.y, _force.z);
  }
}

//////////////////////////////////////////////////
void ODEBody::SetTorque(const math::Vector3 &_torque)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodySetTorque(this->bodyId, _torque.x, _torque.y, _torque.z);
  }
}

//////////////////////////////////////////////////
void ODEBody::AddForce(const math::Vector3 &_force)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodyAddForce(this->bodyId, _force.x, _force.y, _force.z);
  }
}

/////////////////////////////////////////////////
void ODEBody::AddRelativeForce(const math::Vector3 &_force)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodyAddRelForce(this->bodyId, _force.x, _force.y, _force.z);
  }
}

/////////////////////////////////////////////////
void ODEBody::AddForceAtRelativePosition(const math::Vector3 &_force,
                               const math::Vector3 &_relpos)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtRelPos(this->bodyId, _force.x, _force.y, _force.z,
                          _relpos.x, _relpos.y, _relpos.z);
  }
}

/////////////////////////////////////////////////
void ODEBody::AddForceAtWorldPosition(const math::Vector3 &_force,
                                      const math::Vector3 &_pos)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtPos(this->bodyId, _force.x, _force.y, _force.z,
                          _pos.x, _pos.y, _pos.z);
  }
}

/////////////////////////////////////////////////
void ODEBody::AddTorque(const math::Vector3 &_torque)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodyAddTorque(this->bodyId, _torque.x, _torque.y, _torque.z);
  }
}

/////////////////////////////////////////////////
void ODEBody::AddRelativeTorque(const math::Vector3 &_torque)
{
  if (this->bodyId)
  {
    this->SetEnabled(true);
    dBodyAddRelTorque(this->bodyId, _torque.x, _torque.y, _torque.z);
  }
}

/////////////////////////////////////////////////
math::Vector3 ODEBody::GetWorldForce() const
{
  math::Vector3 force;

  if (this->bodyId)
  {
    const dReal *dforce;

    dforce = dBodyGetForce(this->bodyId);

    force.x = dforce[0];
    force.y = dforce[1];
    force.z = dforce[2];
  }

  return force;
}

//////////////////////////////////////////////////
math::Vector3 ODEBody::GetWorldTorque() const
{
  math::Vector3 torque;

  if (this->bodyId)
  {
    const dReal *dtorque;

    dtorque = dBodyGetTorque(this->bodyId);

    torque.x = dtorque[0];
    torque.y = dtorque[1];
    torque.z = dtorque[2];
  }

  return torque;
}

//////////////////////////////////////////////////
dSpaceID ODEBody::GetSpaceId() const
{
  return this->spaceId;
}

//////////////////////////////////////////////////
void ODEBody::SetSpaceId(dSpaceID _spaceid)
{
  this->spaceId = _spaceid;
}

//////////////////////////////////////////////////
void ODEBody::SetLinearDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetLinearDamping(this->GetODEId(), _damping);
}

//////////////////////////////////////////////////
void ODEBody::SetAngularDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetAngularDamping(this->GetODEId(), _damping);
}

//////////////////////////////////////////////////
void ODEBody::SetKinematic(const bool &_state)
{
  this->sdf->GetAttribute("kinematic")->Set(_state);
  if (this->bodyId)
  {
    if (_state)
      dBodySetKinematic(this->bodyId);
    else
      dBodySetDynamic(this->bodyId);
  }
}

//////////////////////////////////////////////////
bool ODEBody::GetKinematic() const
{
  bool result = false;

  if (this->bodyId)
    result = dBodyIsKinematic(this->bodyId);

  return result;
}
