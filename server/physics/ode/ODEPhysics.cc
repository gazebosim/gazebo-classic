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
/* Desc: The ODE physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 * SVN: $Id$
 */

#include <assert.h>

#include "Global.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "Vector3.hh"
#include "Geom.hh"
#include "Body.hh"
#include "ContactParams.hh"
#include "Entity.hh"
#include "SliderJoint.hh"
#include "HingeJoint.hh"
#include "Hinge2Joint.hh"
#include "BallJoint.hh"
#include "UniversalJoint.hh"
#include "XMLConfig.hh"
#include "ODEPhysics.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEPhysics::ODEPhysics()
  : PhysicsEngine()
{
  this->worldId = dWorldCreate();

  this->spaceId = dSimpleSpaceCreate(0);

  this->contactGroup = dJointGroupCreate(0);

  this->gravity.x = 0;
  this->gravity.y = -9.8;
  this->gravity.z = 0;

  this->globalCFM = 10e-10;
  this->globalERP = 0.2;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ODEPhysics::~ODEPhysics()
{
  dSpaceDestroy(this->spaceId);
  dWorldDestroy(this->worldId);

  this->spaceId = NULL;
  this->worldId = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Load the ODE engine
void ODEPhysics::Load(XMLConfigNode *node)
{
  node = node->GetChild("ode");

  if (node == NULL)
    gzthrow("Must define a <physics:ode> node in the XML file");

  this->gravity = node->GetVector3("gravity",this->gravity);
  this->stepTime = node->GetDouble("stepTime",this->stepTime);
  this->globalCFM = node->GetDouble("cfm",10e-10,0);
  this->globalERP = node->GetDouble("erp",0.2,0);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the ODE engine
void ODEPhysics::Init()
{
  dWorldSetGravity(this->worldId, this->gravity.x, this->gravity.y, this->gravity.z);
  dWorldSetCFM(this->worldId, this->globalCFM);
  dWorldSetERP(this->worldId, this->globalERP);

}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE engine
void ODEPhysics::Update()
{
  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide( this->spaceId, this, CollisionCallback );

  // Update the dynamical model
  dWorldStep( this->worldId, this->stepTime );
  //dWorldStepFast1( this->worldId, this->stepTime, 20 );
  //dWorldQuickStep(this->worldId, this->stepTime);

  // Very important to clear out the contact group 
  dJointGroupEmpty( this->contactGroup );

}

////////////////////////////////////////////////////////////////////////////////
// Finilize the ODE engine
void ODEPhysics::Fini()
{
}

////////////////////////////////////////////////////////////////////////////////
// Add an entity
void ODEPhysics::AddEntity(Entity *entity)
{
  // Only the top level parent should have a new space
  if (entity->GetParent() == NULL)
  {
    entity->spaceId = dSimpleSpaceCreate(this->spaceId);
  }
  else
  {
    entity->spaceId = entity->GetParent()->spaceId;
  }

  this->entities[entity->GetId()] = entity;
}

////////////////////////////////////////////////////////////////////////////////
// Create a new body
Body *ODEPhysics::CreateBody(Entity *parent)
{
  return new Body(parent, this->worldId);
}

////////////////////////////////////////////////////////////////////////////////
// Create a new joint
Joint *ODEPhysics::CreateJoint(Joint::Type type)
{
  switch (type)
  {
    case Joint::SLIDER:
      return new SliderJoint(this->worldId);
    case Joint::HINGE:
      return new HingeJoint(this->worldId);
    case Joint::HINGE2:
      return new Hinge2Joint(this->worldId);
    case Joint::BALL:
      return new BallJoint(this->worldId);
    case Joint::UNIVERSAL:
      return new UniversalJoint(this->worldId);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Return the space id 
dSpaceID ODEPhysics::GetSpaceId() const
{
  return this->spaceId;
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::CollisionCallback( void *data, dGeomID o1, dGeomID o2)
{
  int i,n;
  ODEPhysics *self;
  Geom *geom1, *geom2;
  dContactGeom contactGeoms[10];
  dContact contactInfo;
  dJointID joint;
  int num;

  self = (ODEPhysics*) data;
  
  // Maximum number of contacts
  num = sizeof(contactGeoms) / sizeof(contactGeoms[0]);

  // If either geom is a space...
  if (dGeomIsSpace( o1 ) || dGeomIsSpace( o2 ))
  {
    // If the spaces/geoms belong to different spaces, collide them
    if (dGeomGetSpace(o1) != dGeomGetSpace(o2))
      dSpaceCollide2( o1, o2, self, &CollisionCallback );

    // If the spaces belong the world space, collide them
    else if (dGeomGetSpace(o1) == self->spaceId || dGeomGetSpace(o2) == self->spaceId)
      dSpaceCollide2( o1, o2, self, &CollisionCallback );
  }
  else
  {
    // There should be no geoms in the world space
    assert(dGeomGetSpace(o1) != self->spaceId);
    assert(dGeomGetSpace(o2) != self->spaceId);

    // We should never test two geoms in the same space
    assert(dGeomGetSpace(o1) != dGeomGetSpace(o2));

    // Get pointers to the underlying geoms
    geom1 = NULL;
    if (dGeomGetClass(o1) == dGeomTransformClass)
      geom1 = (Geom*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      geom1 = (Geom*) dGeomGetData(o1);

    geom2 = NULL;
    if (dGeomGetClass(o2) == dGeomTransformClass)
      geom2 = (Geom*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      geom2 = (Geom*) dGeomGetData(o2);

    assert(geom1 && geom2);

//    std::cout << "Geom1[" << geom1->GetName() << "] Geom2[" << geom2->GetName() << "]\n";

    /*
    if (geom1->IsStatic() && geom2->IsStatic())
      printf("Geoms are static\n");
      */
    
    // Detect collisions betweed geoms
    n = dCollide(o1, o2, num, contactGeoms, sizeof(contactGeoms[0]));

    for (i=0; i < n; i++)
    {
      dBodyID body1 = dGeomGetBody(contactGeoms[i].g1);
      dBodyID body2 = dGeomGetBody(contactGeoms[i].g2);

      // Dont add contact joints between already connected bodies.
      // Sometimes the body is unspecified; should probably figure out
      // what this means
      if (body1 && body2)
        if (dAreConnectedExcluding(body1, body2, dJointTypeContact))
          continue;
         
      contactInfo.geom = contactGeoms[i];
      contactInfo.surface.mode = 0;
      
      // Compute the CFM and ERP by assuming the two bodies form a
      // spring-damper system.
      double h, kp, kd;
      h = self->stepTime;
      kp = 1 / (1 / geom1->contact->kp + 1 / geom2->contact->kp);
      kd = geom1->contact->kd + geom2->contact->kd;
      contactInfo.surface.mode |= dContactSoftERP | dContactSoftCFM;
      contactInfo.surface.soft_erp = h * kp / (h * kp + kd);
      contactInfo.surface.soft_cfm = 1 / (h * kp + kd);

      // Compute friction effects; this is standard Coulomb friction
      contactInfo.surface.mode |= dContactApprox1;
      contactInfo.surface.mu = MIN(geom1->contact->mu1, geom2->contact->mu1);
      

     /* contactInfo.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;

      contactInfo.surface.soft_erp = 0.8;
      contactInfo.surface.soft_cfm = 0.01;
      contactInfo.surface.slip1 = 0.0;
      contactInfo.surface.slip2 = 0.0;
      contactInfo.surface.mu = 1;
      */

      // Compute slipping effects
      //contactInfo.surface.slip1 = (geom1->contact->slip1 + geom2->contact->slip1)/2.0;
      //contactInfo.surface.slip2 = (geom1->contact->slip2 + geom2->contact->slip2)/2.0;
      
      // Construct a contact joint between the two bodies
      joint = dJointCreateContact(self->worldId, self->contactGroup, &contactInfo);
      dJointAttach(joint, body1, body2);
    }
  }
} 
                                  
