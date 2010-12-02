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

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>


#include "Timer.hh"
#include "PhysicsFactory.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "Vector3.hh"
#include "ODEGeom.hh"
#include "ODEBody.hh"
#include "Entity.hh"
#include "XMLConfig.hh"
#include "SurfaceParams.hh"

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
#include "ODEHeightmapShape.hh"
#include "MapShape.hh"

#include "gazebo_config.h"

#include "ODEPhysics.hh"

using namespace gazebo;

GZ_REGISTER_PHYSICS_ENGINE("ode", ODEPhysics);

class ContactUpdate_TBB
{
  public: ContactUpdate_TBB(tbb::concurrent_vector<ContactFeedback> *contacts) 
          : contacts(contacts) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    std::vector<dJointFeedback>::iterator jiter;

    for (size_t i=r.begin(); i != r.end(); i++)
    {
      ContactFeedback *feedback = &(*this->contacts)[i];

      if (feedback->contact.geom1 == NULL)
        gzerr(0) << "collision update Geom1 is null\n";

      if (feedback->contact.geom2 == NULL)
        gzerr(0) << "Collision update Geom2 is null\n";

      feedback->contact.forces.clear();

      // Copy all the joint forces to the contact
      for (jiter = feedback->feedbacks.begin(); 
          jiter != feedback->feedbacks.end(); jiter++)
      {
        JointFeedback joint;
        joint.body1Force.Set( (*jiter).f1[0], (*jiter).f1[1], (*jiter).f1[2] );
        joint.body2Force.Set( (*jiter).f2[0], (*jiter).f2[1], (*jiter).f2[2] );

        joint.body1Torque.Set((*jiter).t1[0], (*jiter).t1[1], (*jiter).t1[2]);
        joint.body2Torque.Set((*jiter).t2[0], (*jiter).t2[1], (*jiter).t2[2]);

        feedback->contact.forces.push_back(joint);
      }

      // Add the contact to each geom
      feedback->contact.geom1->AddContact( feedback->contact );
      feedback->contact.geom2->AddContact( feedback->contact );
    }
  }

  tbb::concurrent_vector<ContactFeedback> *contacts;
};

class Colliders_TBB
{
  public: Colliders_TBB(
              std::vector< std::pair<ODEGeom*, ODEGeom*> > *colliders, 
              ODEPhysics*engine) : 
    colliders(colliders), engine(engine)
  { 
    dAllocateODEDataForThread(dAllocateMaskAll);
  }

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      ODEGeom *geom1 = (*this->colliders)[i].first;
      ODEGeom *geom2 = (*this->colliders)[i].second;
      this->engine->Collide(geom1, geom2);
    }
  }

  private: std::vector< std::pair<ODEGeom*, ODEGeom*> > *colliders;
  private: ODEPhysics*engine;
};
 
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
  dHashSpaceSetLevels(this->spaceId, -2, 8);

  this->contactGroup = dJointGroupCreate(0);

  // If auto-disable is active, then user interaction with the joints 
  // doesn't behave properly
  dWorldSetAutoDisableFlag(this->worldId, 0);
  dWorldSetAutoDisableTime(this->worldId, 2.0);
  dWorldSetAutoDisableLinearThreshold(this->worldId, 0.001);
  dWorldSetAutoDisableAngularThreshold(this->worldId, 0.001);
  dWorldSetAutoDisableSteps(this->worldId, 50);

  Param::Begin(&this->parameters);

#ifdef QUICKSTEP_EXPERIMENTAL
  /// experimental ode stuff
  this->islandThreadsP = new ParamT<int>("islandThreads",0,0); // number of thread pool threads for islands
  this->quickStepThreadsP = new ParamT<int>("quickStepThreads",0,0); // number of thread pool threads for quickstep
  this->quickStepChunksP = new ParamT<int>("quickStepChunks",1,0); // number of thread pool threads for islands
  this->quickStepOverlapP = new ParamT<int>("quickStepOverlap",0,0); // number of thread pool threads for islands
  this->quickStepToleranceP = new ParamT<double>("quickStepTolerance",0,0); // number of thread pool threads for islands
#endif

  this->globalCFMP = new ParamT<double>("cfm", 10e-5, 0);
  this->globalERPP = new ParamT<double>("erp", 0.2, 0);
  this->stepTypeP = new ParamT<std::string>("stepType", "quick", 0);
  this->stepItersP = new ParamT<unsigned int>("stepIters", 100, 0);
  this->stepWP = new ParamT<double>("stepW", 1.3, 0);  /// over_relaxation value for SOR
  this->contactMaxCorrectingVelP = new ParamT<double>("contactMaxCorrectingVel", 10.0, 0);
  this->contactSurfaceLayerP = new ParamT<double>("contactSurfaceLayer", 0.01, 0);
  this->autoDisableBodyP = new ParamT<bool>("autoDisableBody", false, 0);
  this->contactFeedbacksP = new ParamT<int>("contactFeedbacks", 100, 0); // just an initial value, appears to get resized if limit is breached
  this->maxContactsP = new ParamT<int>("maxContacts",1000,0); // enforced for trimesh-trimesh contacts

  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  this->quickStepP      = new ParamT<bool>  ("quickStep", false, 0, true, "replace quickStep with stepType");
  this->quickStepItersP = new ParamT<int>   ("quickStepIters", -1, 0, true, "replace quickStepIters with stepIters");
  this->quickStepWP     = new ParamT<double>("quickStepW", -1.0, 0, true, "replace quickStepW with stepW");

  Param::End();
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

#ifdef QUICKSTEP_EXPERIMENTAL
  /// experimental ode stuff
  delete this->islandThreadsP;
  delete this->quickStepThreadsP;
  delete this->quickStepChunksP;
  delete this->quickStepOverlapP;
  delete this->quickStepToleranceP;
#endif

  delete this->globalCFMP;
  delete this->globalERPP;
  delete this->stepTypeP;
  delete this->stepItersP;
  delete this->stepWP;
  delete this->contactMaxCorrectingVelP;
  delete this->contactSurfaceLayerP;
  delete this->autoDisableBodyP;
  delete this->contactFeedbacksP;
  delete this->maxContactsP;

  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  delete this->quickStepP;
  delete this->quickStepItersP;
  delete this->quickStepWP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the ODE engine
void ODEPhysics::Load(XMLConfigNode *node)
{
  XMLConfigNode *cnode = NULL;

  if (node)
    cnode = node->GetChild("ode", "physics");

#ifdef QUICKSTEP_EXPERIMENTAL
  /// experimental ode stuff
  this->islandThreadsP->Load(cnode);
  this->quickStepThreadsP->Load(cnode);
  this->quickStepChunksP->Load(cnode);
  this->quickStepOverlapP->Load(cnode);
  this->quickStepToleranceP->Load(cnode);
  dWorldSetIslandThreads(this->worldId, this->islandThreadsP->GetValue() );
  dWorldSetQuickStepThreads(this->worldId, this->quickStepThreadsP->GetValue() );
  dWorldSetQuickStepNumChunks(this->worldId, this->quickStepChunksP->GetValue() );
  dWorldSetQuickStepNumOverlap(this->worldId, this->quickStepOverlapP->GetValue() );
  dWorldSetQuickStepTolerance(this->worldId, this->quickStepToleranceP->GetValue() );
#endif
 
  this->gravityP->Load(cnode);
  this->stepTimeP->Load(cnode);
  this->updateRateP->Load(cnode);
  this->globalCFMP->Load(cnode);
  this->globalERPP->Load(cnode);
  this->stepTypeP->Load(cnode);
  this->stepItersP->Load(cnode);
  this->stepWP->Load(cnode);
  this->contactMaxCorrectingVelP->Load(cnode);
  this->contactSurfaceLayerP->Load(cnode);
  this->autoDisableBodyP->Load(cnode);
  this->contactFeedbacksP->Load(cnode);
  this->maxContactsP->Load(cnode);

  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  this->quickStepP->Load(cnode);
  this->quickStepItersP->Load(cnode);
  this->quickStepWP->Load(cnode);

  // Help prevent "popping of deeply embedded object
  dWorldSetContactMaxCorrectingVel(this->worldId, contactMaxCorrectingVelP->GetValue());

  // This helps prevent jittering problems.
  dWorldSetContactSurfaceLayer(this->worldId, contactSurfaceLayerP->GetValue());

  // If auto-disable is active, then user interaction with the joints 
  // doesn't behave properly
  // disable autodisable by default
  dWorldSetAutoDisableFlag(this->worldId, this->autoDisableBodyP->GetValue());
  dWorldSetAutoDisableTime(this->worldId, 2.0);
  dWorldSetAutoDisableLinearThreshold(this->worldId, 0.001);
  dWorldSetAutoDisableAngularThreshold(this->worldId, 0.001);
  dWorldSetAutoDisableSteps(this->worldId, 50);

  this->contactFeedbacks.resize(this->contactFeedbacksP->GetValue());

  // Reset the contact pointer
  //this->contactFeedbackIter = this->contactFeedbacks.begin();

  Vector3 g = this->gravityP->GetValue();
  dWorldSetGravity(this->worldId, g.x, g.y, g.z);

  dWorldSetCFM(this->worldId, this->globalCFMP->GetValue());
  dWorldSetERP(this->worldId, this->globalERPP->GetValue());

  dWorldSetQuickStepNumIterations(this->worldId, **this->stepItersP );
  dWorldSetQuickStepW(this->worldId, **this->stepWP );


  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  if (this->quickStepItersP->GetValue() > 0) // only set them if specified
    dWorldSetQuickStepNumIterations(this->worldId, **this->quickStepItersP );
  if (this->quickStepWP->GetValue() > 0) // only set them if specified
    dWorldSetQuickStepW(this->worldId, **this->quickStepWP );

}

////////////////////////////////////////////////////////////////////////////////
// Save the ODE engine
void ODEPhysics::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<physics:ode>\n";
  // experimental ode stuff
#ifdef QUICKSTEP_EXPERIMENTAL
  stream << prefix << "  " << *(this->islandThreadsP) << "\n";
  stream << prefix << "  " << *(this->quickStepThreadsP) << "\n";
  stream << prefix << "  " << *(this->quickStepChunksP) << "\n";
  stream << prefix << "  " << *(this->quickStepOverlapP) << "\n";
  stream << prefix << "  " << *(this->quickStepToleranceP) << "\n";
#endif
  stream << prefix << "  " << *(this->stepTimeP) << "\n";
  stream << prefix << "  " << *(this->gravityP) << "\n";
  stream << prefix << "  " << *(this->updateRateP) << "\n";
  stream << prefix << "  " << *(this->globalCFMP) << "\n";
  stream << prefix << "  " << *(this->globalERPP) << "\n";
  stream << prefix << "  " << *(this->stepTypeP) << "\n";
  stream << prefix << "  " << *(this->stepItersP) << "\n";
  stream << prefix << "  " << *(this->stepWP) << "\n";
  stream << prefix << "  " << *(this->contactMaxCorrectingVelP) << "\n";
  stream << prefix << "  " << *(this->contactSurfaceLayerP) << "\n";
  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  stream << prefix << "  " << *(this->quickStepP) << "\n";
  stream << prefix << "  " << *(this->quickStepItersP) << "\n";
  stream << prefix << "  " << *(this->quickStepWP) << "\n";
  stream << prefix << "</physics:ode>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the ODE engine
void ODEPhysics::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
// Init the engine for threads
void ODEPhysics::InitForThread()
{
  dAllocateODEDataForThread(dAllocateMaskAll);
}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE collisions, create joints
void ODEPhysics::UpdateCollision()
{
  this->colliders.clear();
  this->trimeshColliders.clear();

  // Do collision detection; this will add contacts to the contact group
  //this->LockMutex(); 
  dSpaceCollide( this->spaceId, this, CollisionCallback );
  //this->UnlockMutex(); 

  this->contactFeedbacks.clear();

  tbb::parallel_for( tbb::blocked_range<size_t>(0, this->colliders.size(), 10),
      Colliders_TBB(&this->colliders, this) );

  // Trimesh collision must happen in this thread sequentially
  for (int i=0; i<this->trimeshColliders.size(); i++)
  {
    DIAGNOSTICTIMER(timer("ODEPhysics Collision dSpaceCollide",6));
    //dSpaceCollide( this->spaceId, this, CollisionCallback );

    ODEGeom *geom1 = this->trimeshColliders[i].first;
    ODEGeom *geom2 = this->trimeshColliders[i].second;
    this->Collide(geom1, geom2);
  }

  // Process all the contacts, get the feedback info, and call the geom
  // callbacks
  tbb::parallel_for( tbb::blocked_range<size_t>(0, 
        this->contactFeedbacks.size(), 10), 
        ContactUpdate_TBB(&this->contactFeedbacks) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE engine
void ODEPhysics::UpdatePhysics()
{
  {
    DIAGNOSTICTIMER(timer("ODEPhysics: UpdateCollision",6));
    this->UpdateCollision();
  }

  {
    DIAGNOSTICTIMER(timer("ODEPhysics: LockMutex",6));
    //this->LockMutex(); 
  }

  // Update the dynamical model
  /// \brief @todo: quickStepP used here for backwards compatibility,
  ///        should tick tock deprecation as we switch to nested tags
  {
    DIAGNOSTICTIMER(timer("ODEPhysics: Constraint Solver",6));

    if (**this->stepTypeP == "quick" || **this->quickStepP == true)
      dWorldQuickStep(this->worldId, (**this->stepTimeP).Double());
    else if (**this->stepTypeP == "world")
      dWorldStep( this->worldId, (**this->stepTimeP).Double() );
#ifdef PARALLEL_QUICKSTEP
    else if (**this->stepTypeP == "parallel_quick")
      dWorldParallelQuickStep(this->worldId, (**this->stepTimeP).Double());  
#endif
    else
      gzthrow(std::string("Invalid step type[") + **this->stepTypeP);

    // Very important to clear out the contact group
    dJointGroupEmpty( this->contactGroup );
  }

  //this->UnlockMutex(); 
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
Geom *ODEPhysics::CreateGeom(std::string type, Body *body)
{
  ODEGeom *geom = new ODEGeom(body);
  Shape *shape = NULL;

  if ( type == "sphere")
    shape = new ODESphereShape(geom);
  else if ( type == "plane")
    shape = new ODEPlaneShape(geom);
  else if ( type == "box")
    shape = new ODEBoxShape(geom);
  else if ( type == "cylinder")
    shape = new ODECylinderShape(geom);
  else if ( type == "multiray")
    shape = new ODEMultiRayShape(geom);
  else if ( type == "trimesh")
    shape = new ODETrimeshShape(geom);
  else if ( type == "heightmap")
    shape = new ODEHeightmapShape(geom);
  else if ( type == "map")
    shape = new MapShape(geom);
  else
    gzerr(0) << "Unable to create geom of type["<<type<<"]\n";

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

void ODEPhysics::SetAutoDisableFlag(bool auto_disable)
{
  this->autoDisableBodyP->SetValue(auto_disable);
  dWorldSetAutoDisableFlag(this->worldId, this->autoDisableBodyP->GetValue());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step iterations
void ODEPhysics::SetSORPGSIters(unsigned int iters)
{
  this->stepItersP->SetValue(iters);
  dWorldSetQuickStepNumIterations(this->worldId, **this->stepItersP );
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetSORPGSW(double w)
{
  this->stepWP->SetValue(w);
  dWorldSetQuickStepW(this->worldId, this->stepWP->GetValue() );
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetWorldCFM(double cfm)
{
  this->globalCFMP->SetValue(cfm);
  dWorldSetCFM(this->worldId, this->globalCFMP->GetValue() );
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetWorldERP(double erp)
{
  this->globalERPP->SetValue(erp);
  dWorldSetERP(this->worldId, this->globalERPP->GetValue());
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetContactMaxCorrectingVel(double vel)
{
  this->contactMaxCorrectingVelP->SetValue(vel);
  dWorldSetContactMaxCorrectingVel(this->worldId, this->contactMaxCorrectingVelP->GetValue());
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetContactSurfaceLayer(double layer_depth)
{
  this->contactSurfaceLayerP->SetValue(layer_depth);
  dWorldSetContactSurfaceLayer(this->worldId, this->contactSurfaceLayerP->GetValue());
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetMaxContacts(double max_contacts)
{
  this->maxContactsP->SetValue(max_contacts);
}

////////////////////////////////////////////////////////////////////////////////
bool ODEPhysics::GetAutoDisableFlag()
{
  return this->autoDisableBodyP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
int ODEPhysics::GetSORPGSIters()
{
  return this->stepItersP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetSORPGSW()
{
  return this->stepWP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetWorldCFM()
{
  return this->globalCFMP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetWorldERP()
{
  return this->globalERPP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetContactMaxCorrectingVel()
{
  return this->contactMaxCorrectingVelP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetContactSurfaceLayer()
{
  return this->contactSurfaceLayerP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetMaxContacts()
{
  return this->maxContactsP->GetValue();
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
Joint *ODEPhysics::CreateJoint(std::string type)
{
  if (type == "slider")
    return new ODESliderJoint(this->worldId);
  if (type == "hinge")
    return new ODEHingeJoint(this->worldId);
  if (type == "hinge2")
    return new ODEHinge2Joint(this->worldId);
  if (type == "ball")
    return new ODEBallJoint(this->worldId);
  if (type == "universal")
    return new ODEUniversalJoint(this->worldId);
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the space id
dSpaceID ODEPhysics::GetSpaceId() const
{
  return this->spaceId;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the step type
std::string ODEPhysics::GetStepType() const
{
  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  if (**this->quickStepP) return "quick";

  return **this->stepTypeP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step type
void ODEPhysics::SetStepType(const std::string type)
{
  this->stepTypeP->SetValue(type);

  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  this->quickStepP->SetValue(false); // use new tags
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gavity vector
void ODEPhysics::SetGravity(const gazebo::Vector3 &gravity)
{
  this->gravityP->SetValue(gravity);
  dWorldSetGravity(this->worldId, gravity.x, gravity.y, gravity.z);
}

////////////////////////////////////////////////////////////////////////////////
// Handle a collision
void ODEPhysics::CollisionCallback( void *data, dGeomID o1, dGeomID o2)
{
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  // exit without doing anything if the two bodies are connected by a joint
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
    return;

  ODEPhysics *self;
  self = (ODEPhysics*) data;

  // Check if either are spaces
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
    dSpaceCollide2(o1, o2, self, &CollisionCallback);
  }
  else
  {
    ODEGeom *geom1 = NULL;
    ODEGeom *geom2 = NULL;
    /*if (b1 && b2)
    {
      ODEBody *odeBody1 = (ODEBody*)dBodyGetData(b1);
      ODEBody *odeBody2 = (ODEBody*)dBodyGetData(b1);
      if (odeBody1->IsStatic() && odeBody2->IsStatic())
        return;
    }*/

    // Exit if both bodies are not enabled
    if ( (b1 && b2 && !dBodyIsEnabled(b1) && !dBodyIsEnabled(b2)) || 
         (!b2 && b1 && !dBodyIsEnabled(b1)) || 
         (!b1 && b2 && !dBodyIsEnabled(b2)) )
    {
      return;
    }

    // Get pointers to the underlying geoms
    if (dGeomGetClass(o1) == dGeomTransformClass)
      geom1 = (ODEGeom*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      geom1 = (ODEGeom*) dGeomGetData(o1);

    if (dGeomGetClass(o2) == dGeomTransformClass)
      geom2 = (ODEGeom*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      geom2 = (ODEGeom*) dGeomGetData(o2);

    if (geom1->GetShapeType() == TRIMESH_SHAPE ||
        geom2->GetShapeType() == TRIMESH_SHAPE )
      self->trimeshColliders.push_back( std::make_pair(geom1, geom2) );
    else
      self->colliders.push_back( std::make_pair(geom1, geom2) );
    return;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Collide two geoms
void ODEPhysics::Collide(ODEGeom *geom1, ODEGeom *geom2)
{
  int numContacts = 100;
  int j;
  int numc = 0;
  dContact contact;

  dContactGeom contactGeoms[**this->maxContactsP];

  // for now, only use maxContacts if both geometries are trimeshes
  // other types of geometries do not need too many contacts
  if (geom1->GetShapeType() == TRIMESH_SHAPE && 
      geom2->GetShapeType() == TRIMESH_SHAPE)
  {
    numContacts = **this->maxContactsP;
  }

  {
    tbb::spin_mutex::scoped_lock lock(this->collideMutex);
    numc = dCollide(geom1->GetGeomId(), geom2->GetGeomId(), 
        numContacts, contactGeoms, sizeof(contactGeoms[0]) );
  }

  if (numc != 0)
  {
    ContactFeedback contactFeedback;

    contactFeedback.contact.Reset();
    contactFeedback.contact.geom1 = geom1;
    contactFeedback.contact.geom2 = geom2;
    contactFeedback.feedbacks.resize(numc);

    double h, kp, kd;
    for (j=0; j<numc; j++)
    {
      // skip negative depth contacts
      if(contactGeoms[j].depth < 0)
        continue;

      contact.geom = contactGeoms[j];
      //contact.geom = self->contactGeoms[i];

      contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;

      //contact.surface.mode = dContactSlip1 | dContactSlip2 | 
      //  dContactSoftERP | dContactSoftCFM |  
      //  dContactBounce | dContactMu2 | dContactApprox1;

      //contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
      // with dContactSoftERP | dContactSoftCFM the test_pr2_collision overshoots the cup

      // Compute the CFM and ERP by assuming the two bodies form a
      // spring-damper system.
      h = (**this->stepTimeP).Double();
      kp = 1.0 / (1.0 / geom1->surface->kp + 1.0 / geom2->surface->kp);
      kd = geom1->surface->kd + geom2->surface->kd;
      contact.surface.soft_erp = h * kp / (h * kp + kd);
      contact.surface.soft_cfm = 1.0 / (h * kp + kd);

      if (geom1->surface->enableFriction && geom2->surface->enableFriction)
      {
        contact.surface.mu = std::min(geom1->surface->mu1, 
            geom2->surface->mu1);
        contact.surface.mu2 = std::min(geom1->surface->mu2, 
            geom2->surface->mu2);
        contact.surface.slip1 = std::min(geom1->surface->slip1, 
            geom2->surface->slip1);
        contact.surface.slip2 = std::min(geom1->surface->slip2, 
            geom2->surface->slip2);
      }
      else
      {
        contact.surface.mu = 0; 
        contact.surface.mu2 = 0;
        contact.surface.slip1 = 0.1;
        contact.surface.slip2 = 0.1;
      }

      contact.surface.bounce = std::min(geom1->surface->bounce, 
          geom2->surface->bounce);
      contact.surface.bounce_vel = std::min(geom1->surface->bounceVel, 
          geom2->surface->bounceVel);

      dJointID c;

      Vector3 contactPos;
      Vector3 contactNorm;

      {
        tbb::spin_mutex::scoped_lock lock(this->collideMutex);
        c = dJointCreateContact (this->worldId, this->contactGroup, &contact);

        contactPos.Set(contact.geom.pos[0], contact.geom.pos[1], 
            contact.geom.pos[2]);
        contactNorm.Set(contact.geom.normal[0], contact.geom.normal[1], 
            contact.geom.normal[2]);

        this->AddContactVisual( contactPos, contactNorm );
      }


      // Store the contact info 
      if (geom1->GetContactsEnabled() ||
          geom2->GetContactsEnabled())
      {
        contactFeedback.contact.depths.push_back(
            contact.geom.depth);
        contactFeedback.contact.positions.push_back(contactPos);
        contactFeedback.contact.normals.push_back(contactNorm);
        contactFeedback.contact.time = 
          Simulator::Instance()->GetSimTime();
        dJointSetFeedback(c, &(contactFeedback.feedbacks[j]));
      }

      dBodyID b1 = dGeomGetBody(geom1->GetGeomId());
      dBodyID b2 = dGeomGetBody(geom2->GetGeomId());

      dJointAttach (c, b1, b2);
    }

    if (geom1->GetContactsEnabled() || geom2->GetContactsEnabled())
    {
      this->contactFeedbacks.push_back( contactFeedback );
    }
  }
}
