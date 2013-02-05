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
#include "physics/ode/ODELink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODELink::ODELink(EntityPtr _parent)
    : Link(_parent)
{
  this->linkId = NULL;
}

//////////////////////////////////////////////////
ODELink::~ODELink()
{
  if (this->linkId)
    dBodyDestroy(this->linkId);
  this->linkId = NULL;
}

//////////////////////////////////////////////////
void ODELink::Load(sdf::ElementPtr _sdf)
{
  this->odePhysics = boost::shared_dynamic_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void ODELink::Init()
{
  if (!this->IsStatic())
  {
    this->linkId = dBodyCreate(this->odePhysics->GetWorldId());
    dBodySetData(this->linkId, this);

    // Only use auto disable if no joints and no sensors are present
    if (this->GetModel()->GetAutoDisable() &&
        this->GetModel()->GetJointCount() == 0 &&
        this->GetSensorCount() == 0)
    {
      dBodySetAutoDisableDefaults(this->linkId);
      dBodySetAutoDisableFlag(this->linkId, 1);
    }
    else
    {
      dBodySetAutoDisableFlag(this->linkId, 0);
    }
  }

  Link::Init();

  if (this->linkId)
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
          dGeomSetBody(g->GetCollisionId(), this->linkId);

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
          dBodySetMaxVel(this->linkId, g->GetSurface()->maxVel);
          dBodySetMinDepth(this->linkId, g->GetSurface()->minDepth);
        }
      }
    }
  }

  // Update the Center of Mass.
  this->UpdateMass();

  if (this->linkId)
  {
    dBodySetMovedCallback(this->linkId, MoveCallback);
    dBodySetDisabledCallback(this->linkId, DisabledCallback);
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
  math::Vector3 cog = self->dirtyPose.rot.RotateVector(
      self->inertial->GetCoG());

  self->dirtyPose.pos -= cog;

  // TODO: this is an ugly line of code. It's like this for speed.
  self->world->dirtyPoses.push_back(self);

  // self->poseMutex->unlock();
}

//////////////////////////////////////////////////
void ODELink::Fini()
{
  Link::Fini();
  if (this->linkId)
    dBodyDestroy(this->linkId);
  this->linkId = NULL;

  this->odePhysics.reset();
}

//////////////////////////////////////////////////
void ODELink::Update()
{
  Link::Update();
}

//////////////////////////////////////////////////
void ODELink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);
  if (this->linkId)
  {
    dBodySetGravityMode(this->linkId, _mode ? 1: 0);
  }
}

//////////////////////////////////////////////////
bool ODELink::GetGravityMode() const
{
  int mode = 0;
  if (this->linkId)
  {
    mode = dBodyGetGravityMode(this->linkId);
  }

  return mode;
}

//////////////////////////////////////////////////
void ODELink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);
  if (_collide)
    this->spaceId = dSimpleSpaceCreate(this->odePhysics->GetSpaceId());
}

//////////////////////////////////////////////////
void ODELink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->linkId)
    return;

  this->SetEnabled(true);

  const math::Pose myPose = this->GetWorldPose();

  math::Vector3 cog = myPose.rot.RotateVector(this->inertial->GetCoG());

  // adding cog location for ode pose
  dBodySetPosition(this->linkId,
      myPose.pos.x + cog.x,
      myPose.pos.y + cog.y,
      myPose.pos.z + cog.z);

  dQuaternion q;
  q[0] = myPose.rot.w;
  q[1] = myPose.rot.x;
  q[2] = myPose.rot.y;
  q[3] = myPose.rot.z;

  // Set the rotation of the ODE link
  dBodySetQuaternion(this->linkId, q);
}

//////////////////////////////////////////////////
dBodyID ODELink::GetODEId() const
{
  return this->linkId;
}


//////////////////////////////////////////////////
void ODELink::SetEnabled(bool _enable) const
{
  if (!this->linkId)
    return;

  if (_enable)
    dBodyEnable(this->linkId);
  else
    dBodyDisable(this->linkId);
}

/////////////////////////////////////////////////////////////////////
bool ODELink::GetEnabled() const
{
  bool result = true;

  if (this->linkId)
    result = dBodyIsEnabled(this->linkId);

  return result;
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateSurface()
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
        dBodySetMaxVel(this->linkId, g->GetSurface()->maxVel);
        dBodySetMinDepth(this->linkId, g->GetSurface()->minDepth);
      }
    }
  }
}
/////////////////////////////////////////////////////////////////////
void ODELink::UpdateMass()
{
  if (!this->linkId)
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
    dBodySetMass(this->linkId, &odeMass);
  else
    gzthrow("Setting custom link " + this->GetName() + "mass to zero!");
}

//////////////////////////////////////////////////
void ODELink::SetLinearVel(const math::Vector3 &_vel)
{
  if (this->linkId)
  {
    dBodySetLinearVel(this->linkId, _vel.x, _vel.y, _vel.z);
  }
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldLinearVel() const
{
  math::Vector3 vel;

  if (this->linkId)
  {
    dVector3 dvel;
    math::Vector3 cog = this->inertial->GetCoG();
    dBodyGetRelPointVel(this->linkId, -cog.x, -cog.y, -cog.z, dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldLinearVel(const math::Vector3 &_offset) const
{
  math::Vector3 vel;

  if (this->linkId)
  {
    dVector3 dvel;
    math::Vector3 offsetFromCoG = _offset - this->inertial->GetCoG();
    dBodyGetRelPointVel(this->linkId, offsetFromCoG.x, offsetFromCoG.y,
        offsetFromCoG.z, dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldLinearVel(const math::Pose &_pose) const
{
  math::Vector3 vel;

  if (this->linkId)
  {
    dVector3 dvel;
    math::Pose wPose = this->GetWorldPose();
    math::Vector3 offsetFromCoG =
        wPose.rot.RotateVectorReverse(_pose.rot * _pose.pos)
        - this->inertial->GetCoG();
    dBodyGetRelPointVel(this->linkId, offsetFromCoG.x, offsetFromCoG.y,
        offsetFromCoG.z, dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldCoGLinearVel() const
{
  math::Vector3 vel;

  if (this->linkId)
  {
    const dReal *dvel;
    dvel = dBodyGetLinearVel(this->linkId);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
void ODELink::SetAngularVel(const math::Vector3 &_vel)
{
  if (this->linkId)
  {
    dBodySetAngularVel(this->linkId, _vel.x, _vel.y, _vel.z);
  }
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldAngularVel() const
{
  math::Vector3 vel;

  if (this->linkId)
  {
    const dReal *dvel;

    dvel = dBodyGetAngularVel(this->linkId);

    vel.Set(dvel[0], dvel[1], dvel[2]);
  }

  return vel;
}

//////////////////////////////////////////////////
void ODELink::SetForce(const math::Vector3 &_force)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodySetForce(this->linkId, _force.x, _force.y, _force.z);
  }
}

//////////////////////////////////////////////////
void ODELink::SetTorque(const math::Vector3 &_torque)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodySetTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  }
}

//////////////////////////////////////////////////
void ODELink::AddForce(const math::Vector3 &_force)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForce(this->linkId, _force.x, _force.y, _force.z);
  }
}

/////////////////////////////////////////////////
void ODELink::AddRelativeForce(const math::Vector3 &_force)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddRelForce(this->linkId, _force.x, _force.y, _force.z);
  }
}

/////////////////////////////////////////////////
void ODELink::AddForceAtRelativePosition(const math::Vector3 &_force,
                               const math::Vector3 &_relpos)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtRelPos(this->linkId, _force.x, _force.y, _force.z,
                          _relpos.x, _relpos.y, _relpos.z);
  }
}

/////////////////////////////////////////////////
void ODELink::AddForceAtWorldPosition(const math::Vector3 &_force,
                                      const math::Vector3 &_pos)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtPos(this->linkId, _force.x, _force.y, _force.z,
                          _pos.x, _pos.y, _pos.z);
  }
}

/////////////////////////////////////////////////
void ODELink::AddTorque(const math::Vector3 &_torque)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  }
}

/////////////////////////////////////////////////
void ODELink::AddRelativeTorque(const math::Vector3 &_torque)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddRelTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  }
}

/////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldForce() const
{
  math::Vector3 force;

  if (this->linkId)
  {
    const dReal *dforce;

    dforce = dBodyGetForce(this->linkId);

    force.x = dforce[0];
    force.y = dforce[1];
    force.z = dforce[2];
  }

  return force;
}

//////////////////////////////////////////////////
math::Vector3 ODELink::GetWorldTorque() const
{
  math::Vector3 torque;

  if (this->linkId)
  {
    const dReal *dtorque;

    dtorque = dBodyGetTorque(this->linkId);

    torque.x = dtorque[0];
    torque.y = dtorque[1];
    torque.z = dtorque[2];
  }

  return torque;
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
}

//////////////////////////////////////////////////
void ODELink::SetAngularDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetAngularDamping(this->GetODEId(), _damping);
}

//////////////////////////////////////////////////
void ODELink::SetKinematic(const bool &_state)
{
  this->sdf->GetElement("kinematic")->Set(_state);
  if (this->linkId)
  {
    if (_state)
      dBodySetKinematic(this->linkId);
    else
      dBodySetDynamic(this->linkId);
  }
}

//////////////////////////////////////////////////
bool ODELink::GetKinematic() const
{
  bool result = false;

  if (this->linkId)
    result = dBodyIsKinematic(this->linkId);

  return result;
}

//////////////////////////////////////////////////
void ODELink::SetAutoDisable(bool _disable)
{
  if (this->GetModel()->GetJointCount() == 0 && this->linkId)
  {
    dBodySetAutoDisableFlag(this->linkId, _disable);
  }
}
