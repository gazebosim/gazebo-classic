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

#include "common/XMLConfig.hh"
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
// Load the body based on an common::XMLConfig node
void ODEBody::Load(common::XMLConfigNode *node)
{
  Body::Load(node);

  this->odePhysics = boost::shared_dynamic_cast<ODEPhysics>(this->GetWorld()->GetPhysicsEngine());

  if (this->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");
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
    dBodySetAutoDisableFlag(this->bodyId, this->GetAutoDisable());
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
        }
      }
    }
  }
 
  // Update the Center of Mass.
  this->UpdateCoM();

  if (this->bodyId)
  {
    dBodySetMovedCallback(this->bodyId, MoveCallback);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Move callback. Use this to move the visuals
void ODEBody::MoveCallback(dBodyID id)
{
  math::Pose3d pose;
  const dReal *p;
  const dReal *r;
  ODEBody *self = (ODEBody*)(dBodyGetData(id));

  p = dBodyGetPosition(id);
  r = dBodyGetQuaternion(id);

  pose.pos.Set(p[0], p[1], p[2]);
  pose.rot.Set(r[0], r[1], r[2], r[3] );

  self->SetWorldPose(pose);

  /*pose = self->GetRelativePose().GetInverse() + pose;
  pose.Correct();

  self->SetWorldPose(pose);
  */
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

  //math::Pose3d pose = this->comEntity->GetWorldPose();
  math::Pose3d pose = this->GetWorldPose();

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
  bool result = false;

  if (this->bodyId)
    result = dBodyIsEnabled(this->bodyId);

  return result;
}

/////////////////////////////////////////////////////////////////////
// Update the CoM and mass matrix
/*
  What's going on here?  In ODE the CoM of a body corresponds to the
  origin of the body-fixed coordinate system.  In Gazebo, however, we
  want to have arbitrary body coordinate systems (i.e., CoM may be
  displaced from the body-fixed cs).  To get around this limitation in
  ODE, we have an extra fudge-factor (comPose), describing the pose of
  the CoM relative to Gazebo's body-fixed cs.  When using low-level
  ODE functions, one must use apply this factor appropriately.

  The UpdateCoM() function is used to compute this offset, based on
  the mass distribution of attached geoms.  This function also shifts
  the ODE-pose of the geoms, to keep everything in the same place in the
  Gazebo cs.  Simple, neh?

  TODO: messes up if you call it twice; should fix.
*/
void ODEBody::UpdateCoM()
{
  Body::UpdateCoM();

  if (!this->bodyId)
    return;

  if (**this->customMassMatrixP)
  {
    dMass odeMass;
    dMassSetZero(&odeMass);

    // The CoG must always be (0,0,0)
    math::Vector3 cog;

    math::Vector3 principals = this->customMass.GetPrincipalMoments();
    math::Vector3 products = this->customMass.GetProductsofInertia();

    dMassSetParameters(&odeMass, this->customMass.GetAsDouble(),
                       cog.x, cog.y, cog.z,
                       principals.x, principals.y, principals.z,
                       products.x, products.y, products.z);
    if (this->customMass.GetAsDouble() > 0)
      dBodySetMass(this->bodyId, &odeMass);
    else
      gzthrow("Setting custom body " + this->GetName()+"mass to zero!");

    ODEPhysics::ConvertMass(&this->customMass, &odeMass);
  }
  else
  { 
    dMass odeMass;
    ODEPhysics::ConvertMass(&odeMass, this->mass);

    // Center of Gravity must be at (0,0,0) in the body frame
    odeMass.c[0] = 0.0;
    odeMass.c[1] = 0.0;
    odeMass.c[2] = 0.0;

    // Set the mass of the ODE body
    dBodySetMass( this->bodyId, &odeMass );
  }
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

////////////////////////////////////////////////////////////////////////////////
/// Set the auto disable flag.
void ODEBody::SetAutoDisable(const bool &value)
{
  Body::SetAutoDisable(value);
  dBodySetAutoDisableFlag(this->bodyId, this->GetAutoDisable());
}
