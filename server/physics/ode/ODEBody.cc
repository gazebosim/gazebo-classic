/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id: Body.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include <sstream>

#include "XMLConfig.hh"
#include "GazeboMessage.hh"
#include "OgreVisual.hh"

#include "Geom.hh"
#include "ODEGeom.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "PhysicsEngine.hh"
#include "Mass.hh"

#include "ODEBody.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEBody::ODEBody(Entity *parent)
    : Body(parent)
{
  this->odePhysics = dynamic_cast<ODEPhysics*>(this->physicsEngine);

  if (this->odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  if ( !this->IsStatic() )
  {
    this->bodyId = dBodyCreate(this->odePhysics->GetWorldId());
    dBodySetData(this->bodyId, this);
  }
  else
    this->bodyId = NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
ODEBody::~ODEBody()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an XMLConfig node
void ODEBody::Load(XMLConfigNode *node)
{
  Body::Load(node);

  // Update the Center of Mass.
  this->UpdateCoM();

}



////////////////////////////////////////////////////////////////////////////////
// Init the ODE body
void ODEBody::Init() 
{
  Body::Init();

  if (this->bodyId)
  {
    dBodySetMovedCallback(this->bodyId, MoveCallback);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Move callback. Use this to move the visuals
void ODEBody::MoveCallback(dBodyID id)
{
  Pose3d pose;
  const dReal *p;
  const dReal *r;
  ODEBody *self = (ODEBody*)(dBodyGetData(id));

  self->physicsEngine->LockMutex();
  p = dBodyGetPosition(id);
  r = dBodyGetQuaternion(id);

  pose.pos.Set(p[0], p[1], p[2]);
  pose.rot.Set(r[0], r[1], r[2], r[3] );

  self->SetAbsPose(pose, false);
  self->physicsEngine->UnlockMutex();
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
    this->physicsEngine->LockMutex();
    dBodySetGravityMode(this->bodyId, mode ? 1: 0);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this body will collide with others in the model
void ODEBody::SetSelfCollide(bool collide)
{
  if (collide)
    this->spaceId = dSimpleSpaceCreate( this->odePhysics->GetSpaceId() );
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void ODEBody::AttachGeom( Geom *geom )
{
  Body::AttachGeom(geom);

  ODEGeom *odeGeom = (ODEGeom*)(geom);

  if ( this->bodyId && odeGeom->IsPlaceable())
  {
    if (odeGeom->GetGeomId())
    {
      this->physicsEngine->LockMutex();
      dGeomSetBody(odeGeom->GetGeomId(), this->bodyId);
      this->physicsEngine->UnlockMutex();
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Change the ode pose
void ODEBody::OnPoseChange()
{
  if (this->bodyId == NULL)
    return;

  Pose3d pose = this->GetAbsPose();
  this->physicsEngine->LockMutex();
  dBodySetPosition(this->bodyId, pose.pos.x, pose.pos.y, pose.pos.z);

  dQuaternion q;
  q[0] = pose.rot.u;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  // Set the rotation of the ODE body
  dBodySetQuaternion(this->bodyId, q);
  this->physicsEngine->UnlockMutex();
}

// Return the position of the body. in global CS
Vector3 ODEBody::GetPositionRate() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *v;

    this->physicsEngine->LockMutex();
    v = dBodyGetLinearVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = v[0];
    vel.y = v[1];
    vel.z = v[2];
  }
  else
  {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
  }

  return vel;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern ODEBody::GetRotationRate() const
{
  Quatern velQ;
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *v;

    this->physicsEngine->LockMutex();
    v = dBodyGetAngularVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = v[0];
    vel.y = v[1];
    vel.z = v[2];

    velQ.SetFromEuler(vel);
  }
  else
  {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
    velQ.SetFromEuler(vel);
  }

  return velQ;
}

////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Vector3 ODEBody::GetEulerRate() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *v;

    this->physicsEngine->LockMutex();
    v = dBodyGetAngularVel(this->bodyId);
    this->physicsEngine->UnlockMutex();
    vel.x = v[0];
    vel.y = v[1];
    vel.z = v[2];

  }
  else
  {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of this body
dBodyID ODEBody::GetODEId() const
{
  return this->bodyId;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this body is enabled
void ODEBody::SetEnabled(bool enable) const
{
  if (!this->bodyId)
    return;

  this->physicsEngine->LockMutex();

  if (enable)
    dBodyEnable(this->bodyId);
  else
    dBodyDisable(this->bodyId);

  this->physicsEngine->UnlockMutex();
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
    this->physicsEngine->LockMutex();
    dMass odeMass;
    dMassSetZero(&odeMass);

    // The CoG must always be (0,0,0)
    Vector3 cog;

    Vector3 principals = this->customMass.GetPrincipalMoments();
    Vector3 products = this->customMass.GetProductsofInertia();

    dMassSetParameters(&odeMass, this->customMass.GetAsDouble(),
                       cog.x, cog.y, cog.z,
                       principals.x, principals.y, principals.z,
                       products.x, products.y, products.z);
    if (this->customMass.GetAsDouble() > 0)
      dBodySetMass(this->bodyId, &odeMass);
    else
      gzthrow("Setting custom body " + this->GetName()+"mass to zero!");

    this->physicsEngine->ConvertMass(&this->customMass, &odeMass);
    this->physicsEngine->UnlockMutex();
  }
  else
  { 
    dMass odeMass;
    this->physicsEngine->ConvertMass(&odeMass, this->mass);

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
void ODEBody::SetLinearVel(const Vector3 &vel)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetLinearVel(this->bodyId, vel.x, vel.y, vel.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 ODEBody::GetLinearVel() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    this->physicsEngine->LockMutex();
    dvel = dBodyGetLinearVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = dvel[0];
    vel.y = dvel[1];
    vel.z = dvel[2];
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void ODEBody::SetAngularVel(const Vector3 &vel)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetAngularVel(this->bodyId, vel.x, vel.y, vel.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 ODEBody::GetAngularVel() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    this->physicsEngine->LockMutex();
    dvel = dBodyGetAngularVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = dvel[0];
    vel.y = dvel[1];
    vel.z = dvel[2];
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the force applied to the body
void ODEBody::SetForce(const Vector3 &force)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodyAddForce(this->bodyId, force.x, force.y, force.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the force applied to the body
Vector3 ODEBody::GetForce() const
{
  Vector3 force;

  if (this->bodyId)
  {
    const dReal *dforce;

    this->physicsEngine->LockMutex();
    dforce = dBodyGetForce(this->bodyId);
    this->physicsEngine->UnlockMutex();

    force.x = dforce[0];
    force.y = dforce[1];
    force.z = dforce[2];
  }

  return force;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the body
void ODEBody::SetTorque(const Vector3 &torque)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetTorque(this->bodyId, torque.x, torque.y, torque.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the torque applied to the body
Vector3 ODEBody::GetTorque() const
{
  Vector3 torque;

  if (this->bodyId)
  {
    const dReal *dtorque;

    this->physicsEngine->LockMutex();
    dtorque = dBodyGetTorque(this->bodyId);
    this->physicsEngine->UnlockMutex();

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
  dBodySetLinearDamping(this->GetODEId(), damping); 
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular damping factor
void ODEBody::SetAngularDamping(double damping)
{
  dBodySetAngularDamping(this->GetODEId(), damping); 
}

