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

#include "PhysicsFactory.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "Vector3.hh"
#include "ODEGeom.hh"
#include "ODEBody.hh"
#include "ContactParams.hh"
#include "Entity.hh"
#include "XMLConfig.hh"

#include "ODEHingeJoint.hh"
#include "ODEHinge2Joint.hh"
#include "ODESliderJoint.hh"
#include "ODEBallJoint.hh"
#include "ODEUniversalJoint.hh"

#include "ODEBoxShape.hh"
#include "ODESphereShape.hh"
#include "ODECylinderShape.hh"
#include "ODEPlaneShape.hh"
#include "ODETrimeshShape.hh"
#include "ODEMultiRayShape.hh"

#include "ODEPhysics.hh"

#ifdef TIMING
#include "Simulator.hh"// for timing
#endif

using namespace gazebo;

GZ_REGISTER_PHYSICS_ENGINE("ode", ODEPhysics);

////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEPhysics::ODEPhysics()
    : PhysicsEngine()
{
  // Collision detection init
  dInitODE2(0);

  dAllocateODEDataForThread(dAllocateMaskAll);

  this->worldId = dWorldCreate();

  //this->spaceId = dSimpleSpaceCreate(0);
  this->spaceId = dHashSpaceCreate(0);

  this->contactGroup = dJointGroupCreate(0);

  // If auto-disable is active, then user interaction with the joints 
  // doesn't behave properly
  /*dWorldSetAutoDisableFlag(this->worldId, 1);
  dWorldSetAutoDisableTime(this->worldId, 2.0);
  */

  Param::Begin(&this->parameters);
  this->globalCFMP = new ParamT<double>("cfm", 10e-5, 0);
  this->globalERPP = new ParamT<double>("erp", 0.2, 0);
  this->quickStepP = new ParamT<bool>("quickStep", false, 0);
  this->quickStepItersP = new ParamT<int>("quickStepIters", 20, 0);
  this->quickStepWP = new ParamT<double>("quickStepW", 1.3, 0);  /// over_relaxation value for SOR
  this->contactMaxCorrectingVelP = new ParamT<double>("contactMaxCorrectingVel", 10.0, 0);
  this->contactSurfaceLayerP = new ParamT<double>("contactSurfaceLayer", 0.01, 0);
  Param::End();

  //this->contactFeedbacks.resize(100);

  // Reset the contact pointer
  //this->contactFeedbackIter = this->contactFeedbacks.begin();
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
ODEPhysics::~ODEPhysics()
{
  dCloseODE();

  if (this->spaceId)
    dSpaceDestroy(this->spaceId);

  if (this->worldId)
    dWorldDestroy(this->worldId);

  this->spaceId = NULL;
  this->worldId = NULL;

  delete this->globalCFMP;
  delete this->globalERPP;
  delete this->quickStepP;
  delete this->quickStepItersP;
  delete this->quickStepWP;
  delete this->contactMaxCorrectingVelP;
  delete this->contactSurfaceLayerP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the ODE engine
void ODEPhysics::Load(XMLConfigNode *node)
{
  XMLConfigNode *cnode = node->GetChild("ode", "physics");
  if (cnode == NULL)
    gzthrow("Must define a <physics:ode> node in the XML file");

  this->gravityP->Load(cnode);
  this->stepTimeP->Load(cnode);
  this->updateRateP->Load(cnode);
  this->globalCFMP->Load(cnode);
  this->globalERPP->Load(cnode);
  this->quickStepP->Load(cnode);
  this->quickStepItersP->Load(cnode);
  this->quickStepWP->Load(cnode);
  this->contactMaxCorrectingVelP->Load(cnode);
  this->contactSurfaceLayerP->Load(cnode);

  // Help prevent "popping of deeply embedded object
  dWorldSetContactMaxCorrectingVel(this->worldId, contactMaxCorrectingVelP->GetValue());

  // This helps prevent jittering problems.
  dWorldSetContactSurfaceLayer(this->worldId, contactSurfaceLayerP->GetValue());

}

////////////////////////////////////////////////////////////////////////////////
// Save the ODE engine
void ODEPhysics::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<physics:ode>\n";
  stream << prefix << "  " << *(this->stepTimeP) << "\n";
  stream << prefix << "  " << *(this->gravityP) << "\n";
  stream << prefix << "  " << *(this->updateRateP) << "\n";
  stream << prefix << "  " << *(this->globalCFMP) << "\n";
  stream << prefix << "  " << *(this->globalERPP) << "\n";
  stream << prefix << "  " << *(this->quickStepP) << "\n";
  stream << prefix << "  " << *(this->quickStepItersP) << "\n";
  stream << prefix << "  " << *(this->quickStepWP) << "\n";
  stream << prefix << "  " << *(this->contactMaxCorrectingVelP) << "\n";
  stream << prefix << "  " << *(this->contactSurfaceLayerP) << "\n";
  stream << prefix << "</physics:ode>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the ODE engine
void ODEPhysics::Init()
{
  Vector3 g = this->gravityP->GetValue();
  dWorldSetGravity(this->worldId, g.x, g.y, g.z);
  dWorldSetCFM(this->worldId, this->globalCFMP->GetValue());
  dWorldSetERP(this->worldId, this->globalERPP->GetValue());
  dWorldSetQuickStepNumIterations(this->worldId, this->quickStepItersP->GetValue() );
  dWorldSetQuickStepW(this->worldId, this->quickStepWP->GetValue() );
}

////////////////////////////////////////////////////////////////////////////////
// Initialize for separate thread
void ODEPhysics::InitForThread()
{
  dAllocateODEDataForThread(dAllocateMaskAll);
}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE collisions, create joints
void ODEPhysics::UpdateCollision()
{
#ifdef TIMING
  double tmpT1 = Simulator::Instance()->GetWallTime();
#endif
  
  this->LockMutex(); 
  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide( this->spaceId, this, CollisionCallback );
  this->UnlockMutex(); 

  // Process all the contacts, get the feedback info, and call the geom
  // callbacks
  /*for (std::vector<ContactFeedback>::iterator iter = 
        this->contactFeedbacks.begin(); 
        iter != this->contactFeedbackIter; iter++)
  {

    (*iter).geom1->contact->body1Force.Set(
        (*iter).feedback.f1[0], (*iter).feedback.f1[1], (*iter).feedback.f1[2]);
    (*iter).geom2->contact->body2Force.Set(
        (*iter).feedback.f2[0], (*iter).feedback.f2[1], (*iter).feedback.f2[2]);

    (*iter).geom1->contact->body1Torque.Set(
        (*iter).feedback.t1[0], (*iter).feedback.t1[1], (*iter).feedback.t1[2]);
    (*iter).geom1->contact->body2Torque.Set(
        (*iter).feedback.t2[0], (*iter).feedback.t2[1], (*iter).feedback.t2[2]);

    // Call the geom's contact callbacks
    (*iter).geom1->contact->contactSignal( (*iter).geom1, (*iter).geom2 );
    (*iter).geom2->contact->contactSignal( (*iter).geom2, (*iter).geom1 );
  }

  // Reset the contact pointer
  this->contactFeedbackIter = this->contactFeedbacks.begin();
  */

  //usleep(1000000);
#ifdef TIMING
  double tmpT2 = Simulator::Instance()->GetWallTime();
  std::cout << "      Collision DT (" << tmpT2-tmpT1 << ")" << std::endl;
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE engine
// void ODEPhysics::UpdatePhysics()
// {
// #ifdef TIMING
//   double tmpT1 = Simulator::Instance()->GetWallTime();
// #endif
//
//   // Update the dynamical model
//   if (this->quickStepP->GetValue())
//     dWorldQuickStep(this->worldId, this->stepTimeP->GetValue() );
//   else
//     dWorldStep( this->worldId, this->stepTimeP->GetValue() );
//
// #ifdef TIMING
//   double tmpT3 = Simulator::Instance()->GetWallTime();
//   std::cout << "      ODE step DT (" << tmpT3-tmpT1 << ")" << std::endl;
//   //std::cout << "  Physics Total DT (" << tmpT3-tmpT1 << ")" << std::endl;
// #endif
//
//   // Very important to clear out the contact group
//   dJointGroupEmpty( this->contactGroup );
//
// }

////////////////////////////////////////////////////////////////////////////////
// Update the ODE engine
void ODEPhysics::UpdatePhysics()
{
#ifdef TIMING
  double tmpT1 = Simulator::Instance()->GetWallTime();
#endif
 
  this->LockMutex(); 
  this->UpdateCollision();

  // Do collision detection; this will add contacts to the contact group
  //dSpaceCollide( this->spaceId, this, CollisionCallback );

  //usleep(1000000);
#ifdef TIMING
  double tmpT2 = Simulator::Instance()->GetWallTime();
  std::cout << "      Collision DT (" << tmpT2-tmpT1 << ")" << std::endl;
#endif

  // Update the dynamical model
  if (this->quickStepP->GetValue())
    dWorldQuickStep(this->worldId, this->stepTimeP->GetValue() );
  else
    dWorldStep( this->worldId, this->stepTimeP->GetValue() );

#ifdef TIMING
  double tmpT3 = Simulator::Instance()->GetWallTime();
  std::cout << "      ODE step DT (" << tmpT3-tmpT2 << ")" << std::endl;
  //std::cout << "  Physics Total DT (" << tmpT3-tmpT1 << ")" << std::endl;
#endif

  // Very important to clear out the contact group
  dJointGroupEmpty( this->contactGroup );

  this->UnlockMutex(); 
}


////////////////////////////////////////////////////////////////////////////////
// Finilize the ODE engine
void ODEPhysics::Fini()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Add an entity to the world
void ODEPhysics::AddEntity(Entity *entity)
{
}

////////////////////////////////////////////////////////////////////////////////
// Remove an entity from the physics engine
void ODEPhysics::RemoveEntity(Entity *entity)
{
}

////////////////////////////////////////////////////////////////////////////////
// Create a new body
Body *ODEPhysics::CreateBody(Entity *parent)
{
  if (parent == NULL)
    gzthrow("Body must have a parent\n");

  std::map<std::string, dSpaceID>::iterator iter;
  iter = this->spaces.find(parent->GetName());

  if (iter == this->spaces.end())
    this->spaces[parent->GetName()] = dSimpleSpaceCreate(this->spaceId);

  ODEBody *body = new ODEBody(parent);

  body->SetSpaceId( this->spaces[parent->GetName()] );

  return body;
}

////////////////////////////////////////////////////////////////////////////////
// Create a new geom
Geom *ODEPhysics::CreateGeom(Shape::Type type, Body *body)
{
  ODEGeom *geom = new ODEGeom(body);
  Shape *shape = NULL;

  switch (type)
  {
    case Shape::SPHERE:
      shape = new ODESphereShape(geom);
      break;
    case Shape::PLANE:
      shape = new ODEPlaneShape(geom);
      break;
    case Shape::BOX:
      shape = new ODEBoxShape(geom);
      break;
    case Shape::CYLINDER:
      shape = new ODECylinderShape(geom);
      break;
    case Shape::MULTIRAY:
      shape = new ODEMultiRayShape(geom);
      break;
    case Shape::TRIMESH:
      shape = new ODETrimeshShape(geom);
      break;
    default:
      gzerr(0) << "Unable to create geom of type["<<type<<"]\n";
  }

  return geom;
}

////////////////////////////////////////////////////////////////////////////////
// Get the world id
dWorldID ODEPhysics::GetWorldId()
{
  return this->worldId;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert an odeMass to Mass
void ODEPhysics::ConvertMass(Mass *mass, void *engineMass)
{
  dMass *odeMass = (dMass*)engineMass;

  mass->SetMass(odeMass->mass);
  mass->SetCoG( odeMass->c[0], odeMass->c[1], odeMass->c[2] );
  mass->SetInertiaMatrix( odeMass->I[0*4+0], odeMass->I[1*4+1],
      odeMass->I[2*4+2], odeMass->I[0*4+1],
      odeMass->I[0*4+2], odeMass->I[1*4+2] );
}

////////////////////////////////////////////////////////////////////////////////
/// Convert an odeMass to Mass
void ODEPhysics::ConvertMass(void *engineMass, const Mass &mass)
{
  dMass *odeMass = (dMass*)(engineMass);

  odeMass->mass = mass.GetAsDouble();
  odeMass->c[0] = mass.GetCoG()[0];
  odeMass->c[1] = mass.GetCoG()[1];
  odeMass->c[2] = mass.GetCoG()[2];

  odeMass->I[0*4+0] = mass.GetPrincipalMoments()[0];
  odeMass->I[1*4+1] = mass.GetPrincipalMoments()[1];
  odeMass->I[2*4+2] = mass.GetPrincipalMoments()[2];

  odeMass->I[0*4+1] = mass.GetProductsofInertia()[0];
  odeMass->I[0*4+2] = mass.GetProductsofInertia()[1];
  odeMass->I[1*4+2] = mass.GetProductsofInertia()[2];
}

////////////////////////////////////////////////////////////////////////////////
// Create a new joint
Joint *ODEPhysics::CreateJoint(Joint::Type type)
{
  switch (type)
  {
    case Joint::SLIDER:
      return new ODESliderJoint(this->worldId);
    case Joint::HINGE:
      return new ODEHingeJoint(this->worldId);
    case Joint::HINGE2:
      return new ODEHinge2Joint(this->worldId);
    case Joint::BALL:
      return new ODEBallJoint(this->worldId);
    case Joint::UNIVERSAL:
      return new ODEUniversalJoint(this->worldId);
    default:
      return NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Return the space id
dSpaceID ODEPhysics::GetSpaceId() const
{
  return this->spaceId;
}

void ODEPhysics::CollisionCallback( void *data, dGeomID o1, dGeomID o2)
{
  ODEPhysics *self;
  ODEGeom *geom1 = NULL;
  ODEGeom *geom2 = NULL;
  int i;
  int numc = 0;
  dContactGeom contactGeoms[64];
  dContact contact;

  self = (ODEPhysics*) data;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);


  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
    return;


  // Check if either are spaces
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
    dSpaceCollide2(o1, o2, self, &CollisionCallback);
  }
  else
  {

    // We should never test two geoms in the same space
    assert(dGeomGetSpace(o1) != dGeomGetSpace(o2));

    // Get pointers to the underlying geoms
    if (dGeomGetClass(o1) == dGeomTransformClass)
      geom1 = (ODEGeom*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      geom1 = (ODEGeom*) dGeomGetData(o1);

    if (dGeomGetClass(o2) == dGeomTransformClass)
      geom2 = (ODEGeom*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      geom2 = (ODEGeom*) dGeomGetData(o2);

    numc = dCollide(o1,o2,64, contactGeoms, sizeof(contactGeoms[0]));
    if (numc != 0)
    {
      for (i=0; i<numc; i++)
      {
        double h, kp, kd;

        if (0)
        std::cout << "dContactGeoms: "
                  << " geom1: " << geom1->GetName()
                  << " geom2: " << geom2->GetName()
                  << " contact points: " << numc
                  << " contact: " << i
                  << " pos: " << contactGeoms[i].pos[0]<<","<< contactGeoms[i].pos[1]<<","<< contactGeoms[i].pos[2]<<","<< contactGeoms[i].pos[3]
                  << " norm: " << contactGeoms[i].normal[0]<<","<< contactGeoms[i].normal[1]<<","<< contactGeoms[i].normal[2]<<","<< contactGeoms[i].normal[3]
                  << " depth: " << contactGeoms[i].depth
                  << std::endl;
        // skip negative depth contacts
        if(contactGeoms[i].depth < 0)
          continue;

        contact.geom = contactGeoms[i];
        //contact.surface.mode = dContactSlip1 | dContactSlip2 | 
        //                       dContactSoftERP | dContactSoftCFM |  
        //                       dContactBounce | dContactMu2 | dContactApprox1;
        contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
        // with dContactSoftERP | dContactSoftCFM the test_pr2_collision overshoots the cup

        // Compute the CFM and ERP by assuming the two bodies form a
        // spring-damper system.
        h = self->stepTimeP->GetValue();
        kp = 1.0 / (1.0 / geom1->contact->kp + 1.0 / geom2->contact->kp);
        kd = geom1->contact->kd + geom2->contact->kd;
        contact.surface.soft_erp = h * kp / (h * kp + kd);
        contact.surface.soft_cfm = 1.0 / (h * kp + kd);

        if (geom1->contact->enableFriction && geom2->contact->enableFriction)
        {
          contact.surface.mu = std::min(geom1->contact->mu1, 
              geom2->contact->mu1);
          contact.surface.mu2 = std::min(geom1->contact->mu2, 
              geom2->contact->mu2);
          contact.surface.slip1 = std::min(geom1->contact->slip1, 
              geom2->contact->slip1);
          contact.surface.slip2 = std::min(geom1->contact->slip2, 
              geom2->contact->slip2);
        }
        else
        {
          contact.surface.mu = 0; 
          contact.surface.mu2 = 0;
          contact.surface.slip1 = 0.1;
          contact.surface.slip2 = 0.1;
        }

        contact.surface.bounce = std::min(geom1->contact->bounce, 
                                     geom2->contact->bounce);
        contact.surface.bounce_vel = std::min(geom1->contact->bounceVel, 
                                         geom2->contact->bounceVel);
        dJointID c = dJointCreateContact (self->worldId,
                                          self->contactGroup, &contact);

        /*if (self->contactFeedbackIter == self->contactFeedbacks.end())
        {
          self->contactFeedbacks.resize( self->contactFeedbacks.size() + 100);
          fprintf(stderr, "Resize\n");
        }
 
        (*self->contactFeedbackIter).geom1 = geom1;
        (*self->contactFeedbackIter).geom2 = geom2;
        dJointSetFeedback(c, &(*self->contactFeedbackIter).feedback);
        self->contactFeedbackIter++;
        */

        dJointAttach (c,b1,b2);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/*void ODEPhysics::CollisionCallback( void *data, dGeomID o1, dGeomID o2)
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

    //std::cout << "Geom1[" << geom1->GetName() << "] Geom2[" << geom2->GetName() << "]\n";

    assert(geom1 && geom2);

    if (geom1->IsStatic() && geom2->IsStatic())
      printf("Geoms are static\n");

    // Detect collisions betweed geoms
    n = dCollide(o1, o2, num, contactGeoms, sizeof(contactGeoms[0]));

    printf("Num COllisions[%d]\n",n);
    for (i=0; i < n; i++)
    {
      dBodyID body1 = dGeomGetBody(contactGeoms[i].g1);
      dBodyID body2 = dGeomGetBody(contactGeoms[i].g2);
      printf("Bodies[%d %d]\n",body1, body2);

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
      contactInfo.surface.mu = std::min(geom1->contact->mu1, geom2->contact->mu1);
      contactInfo.surface.mu2 = 0;
      contactInfo.surface.bounce = 0.1;
      contactInfo.surface.bounce_vel = 0.1;


     // contactInfo.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;

      //contactInfo.surface.soft_erp = 0.8;
      //contactInfo.surface.soft_cfm = 0.01;
      //contactInfo.surface.slip1 = 0.0;
      //contactInfo.surface.slip2 = 0.0;
      //contactInfo.surface.mu = 1;

      // Compute slipping effects
      //contactInfo.surface.slip1 = (geom1->contact->slip1 + geom2->contact->slip1)/2.0;
      //contactInfo.surface.slip2 = (geom1->contact->slip2 + geom2->contact->slip2)/2.0;

    std::cout << "Geom1[" << geom1->GetName() << "] Geom2[" << geom2->GetName() << "]\n";
      // Construct a contact joint between the two bodies
      joint = dJointCreateContact(self->worldId, self->contactGroup, &contactInfo);
      dJointAttach(joint, body1, body2);
    }
  }
}*/

