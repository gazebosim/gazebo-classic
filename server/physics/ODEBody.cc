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

#include "Quatern.hh"
#include "GazeboError.hh"
#include "ODEPhysics.hh"
#include "PhysicsEngine.hh"

#include "Body.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEBody::ODEBody(Entity *parent)
    : Body(parent)
{
  ODEPhysics *odePhysics = dynamic_cast<ODEPhysics*>(this->physicsEngine);
  if (odePhysics == NULL)
    gzthrow("Not using the ode physics engine");

  if ( !this->IsStatic() )
  {
    this->bodyId = dBodyCreate(odePhyics->GetWorldId());

    dMassSetZero( &this->mass );
  }
  else
  {
    this->bodyId = NULL;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
ODEBody::~ODEBody()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an XMLConfig node
void ODEBody::LoadChild(XMLConfigNode *node)
{
  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  if (**this->selfCollideP)
  {
    //std::cout << "setting self collide: " << this->nameP->GetValue() << std::endl;
    ODEPhysics* pe = dynamic_cast<ODEPhysics*>(World::Instance()->GetPhysicsEngine());
    this->spaceId = dSimpleSpaceCreate( pe->spaceId);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Init the ODE body
void ODEBody::InitChild() 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the body
void ODEBody::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void ODEBody::UpdateChild()
{
  if(this->GetId())
  {
    this->physicsEngine->LockMutex();

	  force = this->dampingFactorP->GetValue() * this->mass.mass;
	  vel = this->GetLinearVel();
	  dBodyAddForce(this->GetId(), -((vel.x * fabs(vel.x)) * force), 
                  -((vel.y * fabs(vel.y)) * force), 
                  -((vel.z * fabs(vel.z)) * force));

	  avel = this->GetAngularVel();
	  dBodyAddTorque(this->GetId(), -avel.x * force, -avel.y * force, 
                   -avel.z * force);

    this->physicsEngine->UnlockMutex();
  }
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
// Attach a geom to this body
void ODEBody::AttachGeomChild( Geom *geom )
{
  if ( this->bodyId )
  {
    if (geom->IsPlaceable())
    {
      this->physicsEngine->LockMutex();

      if (geom->GetTransId())
        dGeomSetBody(geom->GetTransId(), this->bodyId);
      else if (geom->GetGeomId())
        dGeomSetBody(geom->GetGeomId(), this->bodyId);

      this->physicsEngine->UnlockMutex();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the body
void ODEBody::SetPosition(const Vector3 &pos)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetPosition(this->bodyId, pos.x, pos.y, pos.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation of the body
void ODEBody::SetRotation(const Quatern &rot)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();

    dQuaternion q;
    q[0] = rot.u;
    q[1] = rot.x;
    q[2] = rot.y;
    q[3] = rot.z;

    // Set the rotation of the ODE body
    dBodySetQuaternion(this->bodyId, q);

    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the position of the body. in global CS
Vector3 ODEBody::GetPosition() const
{
  Vector3 pos;

  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    const dReal *p;

    p = dBodyGetPosition(this->bodyId);

    pos.x = p[0];
    pos.y = p[1];
    pos.z = p[2];

    // check for NaN
    if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z))
    {
      std::cout << "Your simulation has exploded, position of body(" << this->GetName() << ") has NaN(" << pos << ")" << std::endl;
      //pos = this->pose.pos;
      assert(0);
    }

    this->physicsEngine->UnlockMutex();
  }
  else
  {
    pos = this->staticPose.pos;
  }

  return pos;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern ODEBody::GetRotation() const
{
  Quatern rot;

  if (this->bodyId)
  {
    const dReal *r;

    this->physicsEngine->LockMutex();
    r = dBodyGetQuaternion(this->bodyId);
    this->physicsEngine->UnlockMutex();

    rot.u = r[0];
    rot.x = r[1];
    rot.y = r[2];
    rot.z = r[3];

    // check for NaN
    if (std::isnan(rot.u) || std::isnan(rot.x) || std::isnan(rot.y) || std::isnan(rot.z))
    {
      std::cout << "Your simulation has exploded, rotation of body(" << this->GetName() << ") has NaN(" << rot << ")" << std::endl;
      //rot = this->pose.rot;
      assert(0);
    }
  }
  else
  {
    rot = this->staticPose.rot;
  }

  return rot;
}

////////////////////////////////////////////////////////////////////////////////
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
dBodyID ODEBody::GetId() const
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
  if (!this->bodyId)
    return;

  // user can specify custom mass matrix or alternatively, UpdateCoM will calculate CoM for
  // combined mass of all children geometries.
  if (this->customMassMatrix)
  {
    // Old pose for the CoM
    Pose3d oldPose, newPose, tmpPose;

    // oldPose is the last comPose
    // newPose is mass CoM
    oldPose = this->comPose;

    //std::cout << " in UpdateCoM, name: " << this->GetName() << std::endl;
    //std::cout << " in UpdateCoM, comPose or oldPose: " << this->comPose << std::endl;

    // New pose for the CoM
    newPose.pos.x = this->cx;
    newPose.pos.y = this->cy;
    newPose.pos.z = this->cz;

    std::map< std::string, Geom* >::iterator giter;
    // Fixup the poses of the geoms (they are attached to the CoM)
    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      if (giter->second->IsPlaceable())
      {
        // FOR GEOMS:
        // get pose with comPose set to oldPose
        this->comPose = oldPose;
        tmpPose = giter->second->GetPose();

        // get pose with comPose set to newPose
        this->comPose = newPose;
        giter->second->SetPose(tmpPose, false);
      }
    }

    // FOR BODY: Fixup the pose of the CoM (ODE body)
    // get pose with comPose set to oldPose
    this->comPose = oldPose;
    tmpPose = this->GetPose();
    // get pose with comPose set to newPose
    this->comPose = newPose;
    this->SetPose(tmpPose);

    // Settle on the new CoM pose
    this->comPose = newPose;



    // comPose is zero in this case, we'll keep cx, cy, cz
    this->comPose.Reset();

    this->comPose.pos.x = this->cx;
    this->comPose.pos.y = this->cy;
    this->comPose.pos.z = this->cz;

    this->physicsEngine->LockMutex();
    // setup this->mass as well
    dMassSetParameters(&this->mass, this->bodyMass,
                       this->cx, this->cy, this->cz,
                       //0,0,0,
                       this->ixx,this->iyy,this->izz,
                       this->ixy,this->ixz,this->iyz);

    dMassTranslate( &this->mass, -this->cx, -this->cy, -this->cz);

    // dMatrix3 rot;
    // dMassRotate(&this->mass, rot);

    // Set the mass matrix
    if (this->mass.mass > 0)
      dBodySetMass( this->bodyId, &this->mass );

    // std::cout << " c[0] " << this->mass.c[0] << std::endl;
    // std::cout << " c[1] " << this->mass.c[1] << std::endl;
    // std::cout << " c[2] " << this->mass.c[2] << std::endl;
    // std::cout << " I[0] " << this->mass.I[0] << std::endl;
    // std::cout << " I[1] " << this->mass.I[1] << std::endl;
    // std::cout << " I[2] " << this->mass.I[2] << std::endl;
    // std::cout << " I[3] " << this->mass.I[3] << std::endl;
    // std::cout << " I[4] " << this->mass.I[4] << std::endl;
    // std::cout << " I[5] " << this->mass.I[5] << std::endl;
    // std::cout << " I[6] " << this->mass.I[6] << std::endl;
    // std::cout << " I[7] " << this->mass.I[7] << std::endl;
    // std::cout << " I[8] " << this->mass.I[8] << std::endl;

    this->physicsEngine->UnlockMutex();
  }
  else
  {

    // original gazebo subroutine that gathers mass from all geoms and sums into one single mass matrix

    const dMass *lmass;
    std::map< std::string, Geom* >::iterator giter;

    this->physicsEngine->LockMutex();
    // Construct the mass matrix by combining all the geoms
    dMassSetZero( &this->mass );

    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      lmass = giter->second->GetBodyMassMatrix();
      if (giter->second->IsPlaceable() && giter->second->GetGeomId())
      {
        dMassAdd( &this->mass, lmass );
      }
    }

    // Old pose for the CoM
    Pose3d oldPose, newPose, tmpPose;

    // oldPose is the last comPose
    // newPose is mass CoM
    oldPose = this->comPose;

    if (std::isnan(this->mass.c[0]))
      this->mass.c[0] = 0;

    if (std::isnan(this->mass.c[1]))
      this->mass.c[1] = 0;

    if (std::isnan(this->mass.c[2]))
      this->mass.c[2] = 0;

    // New pose for the CoM
    newPose.pos.x = this->mass.c[0];
    newPose.pos.y = this->mass.c[1];
    newPose.pos.z = this->mass.c[2];

    // Fixup the poses of the geoms (they are attached to the CoM)
    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      if (giter->second->IsPlaceable())
      {
        // FOR GEOMS:
        // get pose with comPose set to oldPose
        this->comPose = oldPose;
        tmpPose = giter->second->GetPose();

        // get pose with comPose set to newPose
        this->comPose = newPose;
        giter->second->SetPose(tmpPose, false);
      }
    }

    // FOR BODY: Fixup the pose of the CoM (ODE body)
    // get pose with comPose set to oldPose
    this->comPose = oldPose;
    tmpPose = this->GetPose();
    // get pose with comPose set to newPose
    this->comPose = newPose;
    this->SetPose(tmpPose);


    // Settle on the new CoM pose
    this->comPose = newPose;

    // My Cheap Hack, to put the center of mass at the origin
    this->mass.c[0] = this->mass.c[1] = this->mass.c[2] = 0;

    // Set the mass matrix
    if (this->mass.mass > 0)
      dBodySetMass( this->bodyId, &this->mass );

    this->physicsEngine->UnlockMutex();
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

