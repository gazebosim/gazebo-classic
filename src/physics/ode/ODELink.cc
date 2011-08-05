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
/* Desc: Link class
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
#include "physics/ode/ODELink.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ODELink::ODELink(EntityPtr parent)
    : Link(parent)
{
  this->linkId = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ODELink::~ODELink()
{
  if (this->linkId)
    dBodyDestroy(this->linkId);
  this->linkId = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Load the link
void ODELink::Load( sdf::ElementPtr &_sdf)
{
  this->odePhysics = boost::shared_dynamic_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  Link::Load(_sdf);
}


////////////////////////////////////////////////////////////////////////////////
// Init the ODE link
void ODELink::Init() 
{

  if ( !this->IsStatic() )
  {
    this->linkId = dBodyCreate(this->odePhysics->GetWorldId());

    dBodySetData(this->linkId, this);
    dBodySetAutoDisableDefaults(this->linkId);
    dBodySetAutoDisableFlag(this->linkId, 0);
  }

  Link::Init();

  if (this->linkId)
  {
    Base_V::iterator iter;
    for (iter = this->children.begin(); iter != this->children.end(); iter++)
    {
      if ((*iter)->HasType(Base::GEOM))
      {
        ODEGeomPtr g = boost::shared_static_cast<ODEGeom>(*iter);
        if (g->IsPlaceable() && g->GetGeomId())
        {
          dGeomSetBody(g->GetGeomId(), this->linkId);
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

  if (this->linkId)
  {
    dBodySetMovedCallback(this->linkId, MoveCallback);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Move callback. Use this to move the visuals
void ODELink::MoveCallback(dBodyID id)
{
  math::Pose pose;
  const dReal *p;
  const dReal *r;
  ODELink *self = (ODELink*)(dBodyGetData(id));

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
/// Finalize the link
void ODELink::Fini()
{
  Link::Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Update the link
void ODELink::Update()
{
  Link::Update();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether gravity affects this link
void ODELink::SetGravityMode(bool mode)
{
  if (this->linkId)
  {
    dBodySetGravityMode(this->linkId, mode ? 1: 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the gravity mode
bool ODELink::GetGravityMode()
{
  int mode = 0;
  if (this->linkId)
  {
    mode = dBodyGetGravityMode(this->linkId);
  }

  return mode;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this link will collide with others in the model
void ODELink::SetSelfCollide(bool collide)
{
  if (collide)
    this->spaceId = dSimpleSpaceCreate( this->odePhysics->GetSpaceId() );
}

////////////////////////////////////////////////////////////////////////////////
// Change the ode pose
void ODELink::OnPoseChange()
{
  if (this->linkId == NULL)
    return;
  //this->SetEnabled(true);

  //math::Pose pose = this->comEntity->GetWorldPose();
  math::Pose pose = this->GetWorldPose();

  math::Vector3 cog_vec = pose.rot.RotateVector(this->inertial->GetCoG());

  // adding cog location for ode pose
  pose.pos += cog_vec;
  dBodySetPosition(this->linkId, pose.pos.x, pose.pos.y, pose.pos.z);

  dQuaternion q;
  q[0] = pose.rot.w;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  // Set the rotation of the ODE link
  dBodySetQuaternion(this->linkId, q);
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of this link
dBodyID ODELink::GetODEId() const
{
  return this->linkId;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this link is enabled
void ODELink::SetEnabled(bool /*_enable*/) const
{
  if (!this->linkId)
    return;

/*  if (enable)
    dBodyEnable(this->linkId);
  else
    dBodyDisable(this->linkId);
    */
}

/////////////////////////////////////////////////////////////////////
/// Get whether this link is enabled in the physics engine
bool ODELink::GetEnabled() const
{
  bool result = true;

  if (this->linkId)
    result = dBodyIsEnabled(this->linkId);

  return result;
}

/////////////////////////////////////////////////////////////////////
// Update the mass matrix
void ODELink::UpdateMass()
{
  if (!this->linkId)
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
    dBodySetMass(this->linkId, &odeMass);
  else
    gzthrow("Setting custom link " + this->GetName()+"mass to zero!");
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the link
void ODELink::SetLinearVel(const math::Vector3 &_vel)
{
  if (this->linkId)
  {
    dBodySetLinearVel(this->linkId, _vel.x, _vel.y, _vel.z);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the link in the world frame
math::Vector3 ODELink::GetWorldLinearVel() const
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

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the link
void ODELink::SetAngularVel(const math::Vector3 &vel)
{
  if (this->linkId)
  {
    dBodySetAngularVel(this->linkId, vel.x, vel.y, vel.z);
  }
}



////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the link in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the force applied to the link
void ODELink::SetForce(const math::Vector3 &force)
{
  if (this->linkId)
  {
    dBodyAddForce(this->linkId, force.x, force.y, force.z);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// \brief Get the force applied to the link in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the link
void ODELink::SetTorque(const math::Vector3 &torque)
{
  if (this->linkId)
  {
    dBodyAddRelTorque(this->linkId, torque.x, torque.y, torque.z);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Get the torque applied to the link in the world frame
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

////////////////////////////////////////////////////////////////////////////////
/// Get the bodies space ID
dSpaceID ODELink::GetSpaceId() const
{
  return this->spaceId;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the bodies space ID
void ODELink::SetSpaceId(dSpaceID spaceid)
{
  this->spaceId = spaceid;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear damping factor
void ODELink::SetLinearDamping(double damping)
{
  if (this->GetODEId())
    dBodySetLinearDamping(this->GetODEId(), damping); 
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular damping factor
void ODELink::SetAngularDamping(double damping)
{
  if (this->GetODEId())
    dBodySetAngularDamping(this->GetODEId(), damping); 
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this link is in the kinematic state
void ODELink::SetKinematic(const bool &state)
{
  if (this->linkId)
  {
    if (state)
      dBodySetKinematic(this->linkId);
    else
      dBodySetDynamic(this->linkId);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get whether this link is in the kinematic state
bool ODELink::GetKinematic() const
{
  bool result = false;

  if (this->linkId)
    result = dBodyIsKinematic(this->linkId);

  return result;
}
