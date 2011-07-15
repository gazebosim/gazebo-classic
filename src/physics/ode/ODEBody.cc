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

#include <sstream>
#include <math.h>

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/Geom.hh"
#include "physics/World.hh"
#include "physics/ode/ODEGeom.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODEBody.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEBody::ODEBody(EntityPtr parent)
    : Body(parent)
{
  this->bodyId = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ODEBody::~ODEBody()
{
  if (this->bodyId)
    dBodyDestroy(this->bodyId);
  this->bodyId = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Load the body
void ODEBody::Load( sdf::ElementPtr &_sdf)
{
  this->odePhysics = boost::shared_dynamic_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  Body::Load(_sdf);
}


////////////////////////////////////////////////////////////////////////////////
// Init the ODE body
void ODEBody::Init() 
{

  if ( !this->IsStatic() )
  {
    this->bodyId = dBodyCreate(this->odePhysics->GetWorldId());

    dBodySetData(this->bodyId, this);
    dBodySetAutoDisableDefaults(this->bodyId);
    dBodySetAutoDisableFlag(this->bodyId, 1);
  }

  Body::Init();

  if (this->bodyId)
  {
    Base_V::iterator iter;
    for (iter = this->children.begin(); iter != this->children.end(); iter++)
    {
      if ((*iter)->HasType(Base::GEOM))
      {
        ODEGeomPtr g = boost::shared_static_cast<ODEGeom>(*iter);
        if (g->IsPlaceable() && g->GetGeomId())
        {
          dGeomSetBody(g->GetGeomId(), this->bodyId);
          // update pose immediately
          math::Pose localPose = g->GetRelativePose();
          dQuaternion q;
          q[0] = localPose.rot.w;
          q[1] = localPose.rot.x;
          q[2] = localPose.rot.y;
          q[3] = localPose.rot.z;
          // Set the pose of the encapsulated geom; this is always relative
          // to the CoM
          dGeomSetOffsetPosition(g->GetGeomId(), localPose.pos.x, localPose.pos.y, 
              localPose.pos.z);
          dGeomSetOffsetQuaternion(g->GetGeomId(), q);
        }
      }
    }
  }
 
  // Update the Center of Mass.
  this->UpdateMass();

  if (this->bodyId)
  {
    dBodySetMovedCallback(this->bodyId, MoveCallback);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Move callback. Use this to move the visuals
void ODEBody::MoveCallback(dBodyID id)
{
  math::Pose pose;
  const dReal *p;
  const dReal *r;
  ODEBody *self = (ODEBody*)(dBodyGetData(id));

  p = dBodyGetPosition(id);
  r = dBodyGetQuaternion(id);

  pose.pos.Set(p[0], p[1], p[2]);
  pose.rot.Set(r[0], r[1], r[2], r[3] );

  // subtracting cog location from ode pose
  math::Vector3 cog_vec = pose.rot.RotateVector(self->inertial->GetCoG());
  pose.pos -= cog_vec;

  self->SetWorldPose(pose,false);
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the body
void ODEBody::Fini()
{
  Body::Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void ODEBody::Update()
{
  Body::Update();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether gravity affects this body
void ODEBody::SetGravityMode(bool mode)
{
  if (this->bodyId)
  {
    dBodySetGravityMode(this->bodyId, mode ? 1: 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the gravity mode
bool ODEBody::GetGravityMode()
{
  int mode = 0;
  if (this->bodyId)
  {
    mode = dBodyGetGravityMode(this->bodyId);
  }

  return mode;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this body will collide with others in the model
void ODEBody::SetSelfCollide(bool collide)
{
  if (collide)
    this->spaceId = dSimpleSpaceCreate( this->odePhysics->GetSpaceId() );
}

////////////////////////////////////////////////////////////////////////////////
// Change the ode pose
void ODEBody::OnPoseChange()
{
  if (this->bodyId == NULL)
    return;
  //this->SetEnabled(true);

  //math::Pose pose = this->comEntity->GetWorldPose();
  math::Pose pose = this->GetWorldPose();

  math::Vector3 cog_vec = pose.rot.RotateVector(this->inertial->GetCoG());

  // adding cog location for ode pose
  pose.pos += cog_vec;
  dBodySetPosition(this->bodyId, pose.pos.x, pose.pos.y, pose.pos.z);

  dQuaternion q;
  q[0] = pose.rot.w;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  // Set the rotation of the ODE body
  dBodySetQuaternion(this->bodyId, q);
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of this body
dBodyID ODEBody::GetODEId() const
{
  return this->bodyId;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this body is enabled
void ODEBody::SetEnabled(bool /*_enable*/) const
{
  if (!this->bodyId)
    return;

/*  if (enable)
    dBodyEnable(this->bodyId);
  else
    dBodyDisable(this->bodyId);
    */
}

/////////////////////////////////////////////////////////////////////
/// Get whether this body is enabled in the physics engine
bool ODEBody::GetEnabled() const
{
  bool result = true;

  if (this->bodyId)
    result = dBodyIsEnabled(this->bodyId);

  return result;
}

/////////////////////////////////////////////////////////////////////
// Update the mass matrix
void ODEBody::UpdateMass()
{
  if (!this->bodyId)
    return;

  dMass odeMass;
  dMassSetZero(&odeMass);

  // The CoG must always be (0,0,0)
  math::Vector3 cog(0,0,0);

  math::Vector3 principals = this->inertial->GetPrincipalMoments();
  math::Vector3 products = this->inertial->GetProductsofInertia();

  dMassSetParameters(&odeMass, this->inertial->GetMass(),
      cog.x, cog.y, cog.z,
      principals.x, principals.y, principals.z,
      products.x, products.y, products.z);

  if (this->inertial->GetMass() > 0)
    dBodySetMass(this->bodyId, &odeMass);
  else
    gzthrow("Setting custom body " + this->GetName()+"mass to zero!");
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void ODEBody::SetLinearVel(const math::Vector3 &vel)
{
  if (this->bodyId)
  {
    dBodySetLinearVel(this->bodyId, vel.x, vel.y, vel.z);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void ODEBody::SetAngularVel(const math::Vector3 &vel)
{
  if (this->bodyId)
  {
    dBodySetAngularVel(this->bodyId, vel.x, vel.y, vel.z);
  }
}



////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the body in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the force applied to the body
void ODEBody::SetForce(const math::Vector3 &force)
{
  if (this->bodyId)
  {
    dBodyAddForce(this->bodyId, force.x, force.y, force.z);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// \brief Get the force applied to the body in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the body
void ODEBody::SetTorque(const math::Vector3 &torque)
{
  if (this->bodyId)
  {
    dBodyAddRelTorque(this->bodyId, torque.x, torque.y, torque.z);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Get the torque applied to the body in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// Get the bodies space ID
dSpaceID ODEBody::GetSpaceId() const
{
  return this->spaceId;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the bodies space ID
void ODEBody::SetSpaceId(dSpaceID spaceid)
{
  this->spaceId = spaceid;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear damping factor
void ODEBody::SetLinearDamping(double damping)
{
  if (this->GetODEId())
    dBodySetLinearDamping(this->GetODEId(), damping); 
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular damping factor
void ODEBody::SetAngularDamping(double damping)
{
  if (this->GetODEId())
    dBodySetAngularDamping(this->GetODEId(), damping); 
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this body is in the kinematic state
void ODEBody::SetKinematic(const bool &state)
{
  if (this->bodyId)
  {
    if (state)
      dBodySetKinematic(this->bodyId);
    else
      dBodySetDynamic(this->bodyId);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get whether this body is in the kinematic state
bool ODEBody::GetKinematic() const
{
  bool result = false;

  if (this->bodyId)
    result = dBodyIsKinematic(this->bodyId);

  return result;
}
