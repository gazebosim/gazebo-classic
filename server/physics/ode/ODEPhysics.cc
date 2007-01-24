#include <assert.h>
#include "Global.hh"
#include "Geom.hh"
#include "Body.hh"
#include "ContactParams.hh"
#include "Entity.hh"
#include "SliderJoint.hh"
#include "HingeJoint.hh"
#include "Hinge2Joint.hh"
#include "BallJoint.hh"
#include "UniversalJoint.hh"
#include "ODEPhysics.hh"

// Constructor
ODEPhysics::ODEPhysics()
  : PhysicsEngine()
{
  this->worldId = dWorldCreate();

  this->spaceId = dSimpleSpaceCreate(0);

  this->contactGroup = dJointGroupCreate(0);

  this->stepTime = 0.02;
}

// Destructor
ODEPhysics::~ODEPhysics()
{
  dSpaceDestroy(this->spaceId);
  dWorldDestroy(this->worldId);

  this->spaceId = NULL;
  this->worldId = NULL;
}

// Load the ODE engine
int ODEPhysics::Load()
{
  return 0;
}

// Initialize the ODE engine
int ODEPhysics::Init()
{
  dWorldSetGravity(this->worldId, 0.0, -9.8, 0.0);

  return 0;
}

//Update the ODE engine
int ODEPhysics::Update()
{
  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide( this->spaceId, this, CollisionCallback );

  // Update the dynamical model
  dWorldStep( this->worldId, this->stepTime );
  //dWorldStepFast1( this->worldId, step, 10 );

  // Very important to clear out the contact group 
  dJointGroupEmpty( this->contactGroup );

  return 0;
}

//Finilize the ODE engine
int ODEPhysics::Fini()
{
  return 0;
}

// Add an entity
int ODEPhysics::AddEntity(Entity *entity)
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

// Create a new body
Body *ODEPhysics::CreateBody(Entity *parent)
{
  return new Body(parent, this->worldId);
}

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

      /*
      printf("%f %f %f %f \n",
             kp, kd,
             contactInfo.surface.soft_erp,
             contactInfo.surface.soft_cfm);
      */

      // Compute friction effects; this is standard Coulomb friction
      contactInfo.surface.mode |= dContactApprox1;
      contactInfo.surface.mu = MIN(geom1->contact->mu1, geom2->contact->mu1);

      // Compute slipping effects
      //contactInfo.surface.slip1 = (geom1->contact->slip1 + geom2->contact->slip1)/2.0;
      //contactInfo.surface.slip2 = (geom1->contact->slip2 + geom2->contact->slip2)/2.0;
      
      // Construct a contact joint between the two bodies
      joint = dJointCreateContact(self->worldId, self->contactGroup, &contactInfo);
      dJointAttach(joint, body1, body2);
    }
  }
  return;
}
                                  
                                  
