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
/* Desc: The ODE physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "common/Diagnostics.hh"
#include "common/Global.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Vector3.hh"
#include "common/XMLConfig.hh"
#include "common/Time.hh"

#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/SurfaceParams.hh"
#include "physics/MapShape.hh"

#include "physics/ode/ODEGeom.hh"
#include "physics/ode/ODEBody.hh"
#include "physics/ode/ODEHingeJoint.hh"
#include "physics/ode/ODEHinge2Joint.hh"
#include "physics/ode/ODESliderJoint.hh"
#include "physics/ode/ODEBallJoint.hh"
#include "physics/ode/ODEUniversalJoint.hh"

#include "physics/ode/ODEBoxShape.hh"
#include "physics/ode/ODESphereShape.hh"
#include "physics/ode/ODECylinderShape.hh"
#include "physics/ode/ODEPlaneShape.hh"
#include "physics/ode/ODETrimeshShape.hh"
#include "physics/ode/ODEMultiRayShape.hh"
#include "physics/ode/ODEHeightmapShape.hh"

#include "physics/ode/ODEPhysics.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("ode", ODEPhysics);

class ContactUpdate_TBB
{
  public: ContactUpdate_TBB(tbb::concurrent_vector<ContactFeedback> *contacts) 
          : contacts(contacts) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {

    for (size_t i=r.begin(); i != r.end(); i++)
    {
      this->engine->ProcessContactFeedback((*this->contacts)[i]);
    }
  }

  private: tbb::concurrent_vector<ContactFeedback> *contacts;
  private: ODEPhysics *engine;
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
    dContactGeom contactGeoms[engine->GetMaxContacts()];
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      ODEGeom *geom1 = (*this->colliders)[i].first;
      ODEGeom *geom2 = (*this->colliders)[i].second;
      this->engine->Collide(geom1, geom2, contactGeoms);
    }
  }

  private: std::vector< std::pair<ODEGeom*, ODEGeom*> > *colliders;
  private: ODEPhysics*engine;
};
 
////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEPhysics::ODEPhysics(WorldPtr world)
    : PhysicsEngine(world)
{
  this->contactGeoms = NULL;

  // Collision detection init
  dInitODE2(0);

  dAllocateODEDataForThread(dAllocateMaskAll);

  this->worldId = dWorldCreate();

  //this->spaceId = dSimpleSpaceCreate(0);
  this->spaceId = dHashSpaceCreate(0);
  dHashSpaceSetLevels(this->spaceId, -2, 8);

  this->contactGroup = dJointGroupCreate(0);

  common::Param::Begin(&this->parameters);
  this->globalCFMP = new common::ParamT<double>("cfm", 10e-5, 0);
  this->globalERPP = new common::ParamT<double>("erp", 0.2, 0);
  this->stepTypeP = new common::ParamT<std::string>("step_type", "quick", 0);
  this->stepItersP = new common::ParamT<unsigned int>("step_iters", 100, 0);
  this->stepWP = new common::ParamT<double>("stepW", 1.3, 0);  /// over_relaxation value for SOR
  this->contactMaxCorrectingVelP = new common::ParamT<double>("contact_max_correcting_vel", 10.0, 0);
  this->contactSurfaceLayerP = new common::ParamT<double>("contact_surface_layer", 0.01, 0);
  this->autoDisableBodyP = new common::ParamT<bool>("auto_disable_body", true, 0);
  this->contactFeedbacksP = new common::ParamT<int>("contact_feedbacks", 100, 0); // just an initial value, appears to get resized if limit is breached
  this->maxContactsP = new common::ParamT<int>("max_contacts",100,0); // enforced for trimesh-trimesh contacts
  common::Param::End();

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
  delete this->stepTypeP;
  delete this->stepItersP;
  delete this->stepWP;
  delete this->contactMaxCorrectingVelP;
  delete this->contactSurfaceLayerP;
  delete this->autoDisableBodyP;
  delete this->contactFeedbacksP;
  delete this->maxContactsP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the ODE engine
void ODEPhysics::Load(common::XMLConfigNode *node)
{
  this->gravityP->Load(node);
  this->stepTimeP->Load(node);
  this->updateRateP->Load(node);
  this->globalCFMP->Load(node);
  this->globalERPP->Load(node);
  this->stepTypeP->Load(node);
  this->stepItersP->Load(node);
  this->stepWP->Load(node);
  this->contactMaxCorrectingVelP->Load(node);
  this->contactSurfaceLayerP->Load(node);
  this->autoDisableBodyP->Load(node);
  this->contactFeedbacksP->Load(node);
  this->maxContactsP->Load(node);

  this->stepTimeDouble = (**this->stepTimeP).Double();

  this->contactGeoms = new dContactGeom[this->GetMaxContacts()];

  // Help prevent "popping of deeply embedded object
  dWorldSetContactMaxCorrectingVel(this->worldId, **contactMaxCorrectingVelP);

  // This helps prevent jittering problems.
  dWorldSetContactSurfaceLayer(this->worldId, **contactSurfaceLayerP);

  // If auto-disable is active, then user interaction with the joints 
  // doesn't behave properly
  // disable autodisable by default
  dWorldSetAutoDisableFlag(this->worldId,1);// **this->autoDisableBodyP);
  /*dWorldSetAutoDisableTime(this->worldId, 1);
  dWorldSetAutoDisableLinearThreshold(this->worldId, 0.01);
  dWorldSetAutoDisableAngularThreshold(this->worldId, 0.01);
  dWorldSetAutoDisableSteps(this->worldId, 10);
  */
  this->contactFeedbacks.resize(**this->contactFeedbacksP);

  // NATY: not sure if I can remove this...check
  // Reset the contact pointer
  //this->contactFeedbackIter = this->contactFeedbacks.begin();

  common::Vector3 g = **this->gravityP;
  dWorldSetGravity(this->worldId, g.x, g.y, g.z);

  dWorldSetCFM(this->worldId, **this->globalCFMP);
  dWorldSetERP(this->worldId, **this->globalERPP);

  dWorldSetQuickStepNumIterations(this->worldId, **this->stepItersP );
  dWorldSetQuickStepW(this->worldId, **this->stepWP );

  // Set the physics update function
  if (**this->stepTypeP == "quick")
    this->physicsStepFunc = &dWorldQuickStep;
  else if (**this->stepTypeP == "world")
    this->physicsStepFunc = &dWorldStep;
  else
    gzthrow(std::string("Invalid step type[") + **this->stepTypeP);
}

////////////////////////////////////////////////////////////////////////////////
// Save the ODE engine
void ODEPhysics::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<physics type=\"ode\">\n";
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
  stream << prefix << "</physics>\n";
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
  int i;
  this->colliders.clear();
  this->trimeshColliders.clear();

  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide( this->spaceId, this, CollisionCallback );

  this->contactFeedbacks.clear();

  // Collide all the geoms
  if (this->colliders.size() < 50)
  {
    for (i=0; i<this->colliders.size(); i++)
    {
      this->Collide(this->colliders[i].first, 
                    this->colliders[i].second, this->contactGeoms);
    }
  }
  else
  {
    tbb::parallel_for( tbb::blocked_range<size_t>(0, 
          this->colliders.size(), 10), Colliders_TBB(&this->colliders, this) );
  }

  // Trimesh collision must happen in this thread sequentially
  for (i=0; i<this->trimeshColliders.size(); i++)
  {
    ODEGeom *geom1 = this->trimeshColliders[i].first;
    ODEGeom *geom2 = this->trimeshColliders[i].second;
    this->Collide(geom1, geom2, this->contactGeoms);
  }

  // Process all the contact feedbacks
  if (this->contactFeedbacks.size() < 50)
  {
    for (i=0; i < this->contactFeedbacks.size(); i++)
    {
      this->ProcessContactFeedback( this->contactFeedbacks[i] );
    }
  }
  else
  {
    // Process all the contacts, get the feedback info, and call the geom
    // callbacks
    tbb::parallel_for( tbb::blocked_range<size_t>(0, 
          this->contactFeedbacks.size(), 20), 
        ContactUpdate_TBB(&this->contactFeedbacks) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE engine
void ODEPhysics::UpdatePhysics()
{
  this->UpdateCollision();
  
  // Update the dynamical model
  (*physicsStepFunc)(this->worldId, this->stepTimeDouble);

  // Very important to clear out the contact group
  dJointGroupEmpty( this->contactGroup );
}


////////////////////////////////////////////////////////////////////////////////
// Finilize the ODE engine
void ODEPhysics::Fini()
{
}


////////////////////////////////////////////////////////////////////////////////
// Create a new body
BodyPtr ODEPhysics::CreateBody(EntityPtr parent)
{
  if (parent == NULL)
    gzthrow("Body must have a parent\n");

  std::map<std::string, dSpaceID>::iterator iter;
  iter = this->spaces.find(parent->GetName());

  if (iter == this->spaces.end())
    this->spaces[parent->GetName()] = dSimpleSpaceCreate(this->spaceId);

  ODEBodyPtr body( new ODEBody(parent) );

  body->SetSpaceId( this->spaces[parent->GetName()] );

  return body;
}

////////////////////////////////////////////////////////////////////////////////
// Create a new geom
GeomPtr ODEPhysics::CreateGeom(const std::string &type, BodyPtr body)
{
  ODEGeomPtr geom( new ODEGeom(body) );
  ShapePtr shape;

  if ( type == "sphere")
    shape.reset( new ODESphereShape(geom) );
  else if ( type == "plane")
    shape.reset( new ODEPlaneShape(geom) );
  else if ( type == "box")
    shape.reset( new ODEBoxShape(geom) );
  else if ( type == "cylinder")
    shape.reset( new ODECylinderShape(geom) );
  else if ( type == "multiray")
    shape.reset( new ODEMultiRayShape(geom) );
  else if ( type == "mesh")
    shape.reset( new ODETrimeshShape(geom) );
  else if ( type == "heightmap")
    shape.reset( new ODEHeightmapShape(geom) );
  else if ( type == "map")
    shape.reset( new MapShape(geom) );
  else
    gzerr << "Unable to create geom of type[" << type << "]\n";

  geom->SetShape(shape);

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

  if (this->contactGeoms)
    delete this->contactGeoms;
  
  this->contactGeoms = new dContactGeom[**this->maxContactsP];
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
int ODEPhysics::GetMaxContacts()
{
  return (**this->maxContactsP);
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
JointPtr ODEPhysics::CreateJoint(const std::string &type)
{
  JointPtr joint;

  if (type == "slider")
    joint.reset( new ODESliderJoint(this->worldId) );
  if (type == "hinge")
    joint.reset( new ODEHingeJoint(this->worldId) );
  if (type == "hinge2")
    joint.reset( new ODEHinge2Joint(this->worldId) );
  if (type == "ball")
    joint.reset( new ODEBallJoint(this->worldId) );
  if (type == "universal")
    joint.reset( new ODEUniversalJoint(this->worldId) );

  return joint;
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
  return **this->stepTypeP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step type
void ODEPhysics::SetStepType(const std::string type)
{
  this->stepTypeP->SetValue(type);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gavity vector
void ODEPhysics::SetGravity(const gazebo::common::Vector3 &gravity)
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

  ODEPhysics *self = (ODEPhysics*) data;

  // Check if either are spaces
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
    dSpaceCollide2(o1, o2, self, &CollisionCallback);
  }
  else
  {
    ODEGeom *geom1 = NULL;
    ODEGeom *geom2 = NULL;

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

    if (geom1->GetShapeType() == Base::TRIMESH_SHAPE ||
        geom2->GetShapeType() == Base::TRIMESH_SHAPE )
      self->trimeshColliders.push_back( std::make_pair(geom1, geom2) );
    else
      self->colliders.push_back( std::make_pair(geom1, geom2) );
  }
}


////////////////////////////////////////////////////////////////////////////////
// Collide two geoms
void ODEPhysics::Collide(ODEGeom *geom1, ODEGeom *geom2, 
                         dContactGeom *contactGeoms)
{
  int numContacts = 10;
  int j;
  int numc = 0;
  dContact contact;

  // for now, only use maxContacts if both geometries are trimeshes
  // other types of geometries do not need too many contacts
  if (geom1->GetShapeType() == Base::TRIMESH_SHAPE && 
      geom2->GetShapeType() == Base::TRIMESH_SHAPE)
  {
    numContacts = this->GetMaxContacts();
  }

  {
    tbb::spin_mutex::scoped_lock lock(this->collideMutex);
    numc = dCollide(geom1->GetGeomId(), geom2->GetGeomId(), 
        numContacts, contactGeoms, sizeof(contactGeoms[0]) );
  }

  if (numc != 0)
  {
    /* NATY: Put this functionality back in, but needs to be faster.
    ContactFeedback contactFeedback;

    contactFeedback.contact.Reset();
    contactFeedback.contact.geom1 = geom1;
    contactFeedback.contact.geom2 = geom2;
    contactFeedback.feedbacks.resize(numc);
    */

    double h, kp, kd;
    for (j=0; j<numc; j++)
    {
      // skip negative depth contacts
      if(contactGeoms[j].depth < 0)
        continue;

      contact.geom = contactGeoms[j];

      contact.surface.mode = dContactSlip1 | dContactSlip2 | 
                             dContactSoftERP | dContactSoftCFM |  
                             dContactBounce | dContactMu2 | dContactApprox1;

      // Compute the CFM and ERP by assuming the two bodies form a
      // spring-damper system.
      h = this->stepTimeDouble;
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

      // NATY: Put this functionality back in, but needs to be faster.
      /*common::Vector3 contactPos;
      common::Vector3 contactNorm;
      */

      {
        tbb::spin_mutex::scoped_lock lock(this->collideMutex);
        c = dJointCreateContact (this->worldId, this->contactGroup, &contact);

        /* NATY: Put back in, but improve performance
        contactPos.Set(contact.geom.pos[0], contact.geom.pos[1], 
            contact.geom.pos[2]);
        contactNorm.Set(contact.geom.normal[0], contact.geom.normal[1], 
            contact.geom.normal[2]);

        this->AddContactVisual( contactPos, contactNorm );
        */
      }


      /* NATY: Put back in, but improve performance
      // Store the contact info 
      if (geom1->GetContactsEnabled() ||
          geom2->GetContactsEnabled())
      {
        contactFeedback.contact.depths.push_back(
            contact.geom.depth);
        contactFeedback.contact.positions.push_back(contactPos);
        contactFeedback.contact.normals.push_back(contactNorm);
        contactFeedback.contact.time = 
          this->world->GetSimTime();
        dJointSetFeedback(c, &(contactFeedback.feedbacks[j]));
      }
      */

      dBodyID b1 = dGeomGetBody(geom1->GetGeomId());
      dBodyID b2 = dGeomGetBody(geom2->GetGeomId());

      dJointAttach (c, b1, b2);
    }

    /* NATY: Put back in, but improve performance
    if (geom1->GetContactsEnabled() || geom2->GetContactsEnabled())
    {
      this->contactFeedbacks.push_back( contactFeedback );
    }
    */
  }
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::ProcessContactFeedback(ContactFeedback &feedback)
{
  std::vector<dJointFeedback>::iterator jiter;

  if (feedback.contact.geom1 == NULL)
    gzerr << "collision update Geom1 is null\n";

  if (feedback.contact.geom2 == NULL)
    gzerr << "Collision update Geom2 is null\n";

  feedback.contact.forces.clear();

  // Copy all the joint forces to the contact
  for (jiter = feedback.feedbacks.begin(); 
      jiter != feedback.feedbacks.end(); jiter++)
  {
    JointFeedback joint;
    joint.body1Force.Set( (*jiter).f1[0], (*jiter).f1[1], (*jiter).f1[2] );
    joint.body2Force.Set( (*jiter).f2[0], (*jiter).f2[1], (*jiter).f2[2] );

    joint.body1Torque.Set((*jiter).t1[0], (*jiter).t1[1], (*jiter).t1[2]);
    joint.body2Torque.Set((*jiter).t2[0], (*jiter).t2[1], (*jiter).t2[2]);

    feedback.contact.forces.push_back(joint);
  }

  // Add the contact to each geom
  feedback.contact.geom1->AddContact( feedback.contact );
  feedback.contact.geom2->AddContact( feedback.contact );
}
