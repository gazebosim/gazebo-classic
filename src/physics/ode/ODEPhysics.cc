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

#include "gazebo_config.h"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "common/Diagnostics.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"
#include "common/Time.hh"

#include "transport/Publisher.hh"

#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/Model.hh"
#include "physics/SurfaceParams.hh"
#include "physics/MapShape.hh"

#include "physics/ode/ODECollision.hh"
#include "physics/ode/ODELink.hh"
#include "physics/ode/ODEScrewJoint.hh"
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

GZ_REGISTER_PHYSICS_ENGINE("ode", ODEPhysics)

/*
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
*/

class Colliders_TBB
{
  public: Colliders_TBB(
              std::vector< std::pair<ODECollision*, ODECollision*> > *colliders, 
              ODEPhysics*engine) : 
    colliders(colliders), engine(engine)
  { 
    dAllocateODEDataForThread(dAllocateMaskAll);
  }

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    dContactGeom *contactCollisions = new dContactGeom[this->engine->GetMaxContacts()];

    for (size_t i=r.begin(); i != r.end(); i++)
    {
      ODECollision *collision1 = (*this->colliders)[i].first;
      ODECollision *collision2 = (*this->colliders)[i].second;
      this->engine->Collide(collision1, collision2, contactCollisions);
    }

    delete contactCollisions;
  }

  private: std::vector< std::pair<ODECollision*, ODECollision*> > *colliders;
  private: ODEPhysics *engine;
};
 
////////////////////////////////////////////////////////////////////////////////
// Constructor
ODEPhysics::ODEPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  this->contactCollisions = NULL;

  // Collision detection init
  dInitODE2(0);

  dAllocateODEDataForThread(dAllocateMaskAll);

  this->worldId = dWorldCreate();

  //this->spaceId = dSimpleSpaceCreate(0);
  this->spaceId = dHashSpaceCreate(0);
  dHashSpaceSetLevels(this->spaceId, -2, 8);

  this->contactGroup = dJointGroupCreate(0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ODEPhysics::~ODEPhysics()
{
  dCloseODE();

  for (unsigned int i=0; i < this->contactFeedbacks.size(); i++)
    delete this->contactFeedbacks[i];
  this->contactFeedbacks.clear();

  dJointGroupDestroy(this->contactGroup);

  if (this->spaceId)
  {
    dSpaceSetCleanup(this->spaceId, 0);
    dSpaceDestroy(this->spaceId);
  }

  if (this->worldId)
    dWorldDestroy(this->worldId);

  this->spaceId = NULL;
  this->worldId = NULL;

  delete [] this->contactCollisions;
}

////////////////////////////////////////////////////////////////////////////////
// Load the ODE engine
void ODEPhysics::Load( sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  sdf::ElementPtr odeElem = _sdf->GetElement("ode");

  this->stepTimeDouble = odeElem->GetElement("solver")->GetValueDouble("dt");
  this->stepType = odeElem->GetElement("solver")->GetValueString("type");

  this->contactCollisions = new dContactGeom[this->GetMaxContacts()];

  // Help prevent "popping of deeply embedded object
  dWorldSetContactMaxCorrectingVel(this->worldId, 
      odeElem->GetOrCreateElement("constraints")->GetValueDouble("contact_max_correcting_vel"));

  // This helps prevent jittering problems.
  dWorldSetContactSurfaceLayer(this->worldId,
      odeElem->GetOrCreateElement("constraints")->GetValueDouble("contact_surface_layer"));

  // If auto-disable is active, then user interaction with the joints 
  // doesn't behave properly
  // disable autodisable by default
  dWorldSetAutoDisableFlag(this->worldId,0);

  /*dWorldSetAutoDisableTime(this->worldId, 1);
  dWorldSetAutoDisableLinearThreshold(this->worldId, 0.01);
  dWorldSetAutoDisableAngularThreshold(this->worldId, 0.01);
  dWorldSetAutoDisableSteps(this->worldId, 10);
  */
  
  math::Vector3 g = this->sdf->GetOrCreateElement("gravity")->GetValueVector3("xyz");
  if (g == math::Vector3(0,0,0))
    gzwarn << "Gravity vector is (0,0,0). Objects will float.\n";

  dWorldSetGravity(this->worldId, g.x, g.y, g.z);

  if (odeElem->HasElement("constraints"))
  {
    dWorldSetCFM(this->worldId, 
        odeElem->GetElement("constraints")->GetValueDouble("cfm"));
    dWorldSetERP(this->worldId, 
        odeElem->GetElement("constraints")->GetValueDouble("erp"));
  }
  else
    dWorldSetERP(this->worldId, 0.2);

  dWorldSetQuickStepNumIterations(this->worldId, this->GetSORPGSIters() );
  dWorldSetQuickStepW(this->worldId, this->GetSORPGSW());

  // Set the physics update function
  if (this->stepType == "quick")
    this->physicsStepFunc = &dWorldQuickStep;
  else if (this->stepType == "world")
    this->physicsStepFunc = &dWorldStep;
  else
    gzthrow(std::string("Invalid step type[") + this->stepType);

}

void ODEPhysics::OnRequest( const boost::shared_ptr<msgs::Request const> &_msg )
{
  msgs::Response response;
  response.set_id( _msg->id() );
  response.set_request( _msg->request() );
  response.set_response( "success" );
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info" )
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type( msgs::Physics::ODE );
    physicsMsg.set_solver_type( this->stepType );
    physicsMsg.set_dt( this->stepTimeDouble );
    physicsMsg.set_iters( this->GetSORPGSIters() );
    physicsMsg.set_sor( this->GetSORPGSW() );
    physicsMsg.set_cfm( this->GetWorldCFM() );
    physicsMsg.set_erp( this->GetWorldERP() );
    physicsMsg.set_contact_max_correcting_vel( this->GetContactMaxCorrectingVel() );
    physicsMsg.set_contact_surface_layer( this->GetContactSurfaceLayer() );
    physicsMsg.mutable_gravity()->CopyFrom( msgs::Convert(this->GetGravity()) );

    response.set_type( physicsMsg.GetTypeName() );
    physicsMsg.SerializeToString( serializedData );
  }

  this->responsePub->Publish( response );
}

void ODEPhysics::OnPhysicsMsg( 
    const boost::shared_ptr<msgs::Physics const> &_msg )
{

  if (_msg->has_dt())
  {
    this->SetStepTime(_msg->dt());
  }
  if (_msg->has_solver_type())

  {
    sdf::ElementPtr solverElem = this->sdf->GetOrCreateElement("ode")->GetOrCreateElement("solver");
    if (_msg->solver_type() == "quick")
    {
      solverElem->GetAttribute("type")->Set("quick");
      this->physicsStepFunc = &dWorldQuickStep;
    }
    else if (_msg->solver_type() == "world")
    {
      solverElem->GetAttribute("type")->Set("world");
      this->physicsStepFunc = &dWorldStep;
    }
  }

  if (_msg->has_iters())
    this->SetSORPGSIters(_msg->iters());

  if (_msg->has_sor())
    this->SetSORPGSW(_msg->sor());

  if (_msg->has_cfm())
    this->SetWorldCFM( _msg->cfm() );

  if (_msg->has_erp())
    this->SetWorldERP( _msg->erp() );

  if (_msg->has_contact_max_correcting_vel())
    this->SetContactMaxCorrectingVel( _msg->contact_max_correcting_vel() );

  if (_msg->has_contact_surface_layer())
    this->SetContactSurfaceLayer( _msg->contact_surface_layer() );

  if (_msg->has_gravity())
    this->SetGravity(msgs::Convert(_msg->gravity()) );
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
  this->collidersCount = 0;
  this->trimeshCollidersCount = 0;

  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide( this->spaceId, this, CollisionCallback );

  for (unsigned int i=0; i < this->contactFeedbacks.size(); i++)
    delete this->contactFeedbacks[i];
  this->contactFeedbacks.clear();

  // Collide all the collisions
  //if (this->colliders.size() < 50)
  //if (this->collidersCount < 50)
  //{
    for (unsigned int i=0; i<this->collidersCount; i++)
    {
      this->Collide(this->colliders[i].first, 
                    this->colliders[i].second, this->contactCollisions);
    }
  //}
  //else
  //{
  //  tbb::parallel_for( tbb::blocked_range<size_t>(0, 
  //        this->collidersCount, 10), Colliders_TBB(&this->colliders, this) );
  //}

  // Trimesh collision must happen in this thread sequentially
  /*for (unsigned int i=0; i<this->trimeshCollidersCount; i++)
  {
    ODECollision *collision1 = this->trimeshColliders[i].first;
    ODECollision *collision2 = this->trimeshColliders[i].second;
    this->Collide(collision1, collision2, this->contactCollisions);
  }*/

  //printf("ContactFeedbacks[%d]\n",this->contactFeedbacks.size());
  // Process all the contact feedbacks
  // tbb not has memeory issues
  //if (this->contactFeedbacks.size() < 50)
  //{
/*    for (unsigned int i=0; i < this->contactFeedbacks.size(); i++)
    {
      this->ProcessContactFeedback(this->contactFeedbacks[i]);
    }
    */
    
  //}
  //else
  //{
  //  // Process all the contacts, get the feedback info, and call the collision
  //  // callbacks
  //  tbb::parallel_for( tbb::blocked_range<size_t>(0, 
  //        this->contactFeedbacks.size(), 20), 
  //      ContactUpdate_TBB(&this->contactFeedbacks) );
  //}
}

////////////////////////////////////////////////////////////////////////////////
// Update the ODE engine
void ODEPhysics::UpdatePhysics()
{
  {
    // Update the dynamical model
    (*physicsStepFunc)(this->worldId, this->stepTimeDouble);
  }

    for (unsigned int i=0; i < this->contactFeedbacks.size(); i++)
    {
      this->ProcessContactFeedback(this->contactFeedbacks[i]);
    }
 
  // Very important to clear out the contact group
  dJointGroupEmpty( this->contactGroup );
}


////////////////////////////////////////////////////////////////////////////////
// Finilize the ODE engine
void ODEPhysics::Fini()
{
  PhysicsEngine::Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Set the step time
void ODEPhysics::SetStepTime(double _value)
{
  this->sdf->GetOrCreateElement("ode")->GetOrCreateElement("solver")->GetAttribute("dt")->Set(_value);

  this->stepTimeDouble = _value;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the simulation step time
double ODEPhysics::GetStepTime()
{
  return this->stepTimeDouble;
}
 

////////////////////////////////////////////////////////////////////////////////
// Create a new body
LinkPtr ODEPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  std::map<std::string, dSpaceID>::iterator iter;
  iter = this->spaces.find(_parent->GetName());

  if (iter == this->spaces.end())
    this->spaces[_parent->GetName()] = dSimpleSpaceCreate(this->spaceId);

  ODELinkPtr link( new ODELink(_parent) );

  link->SetSpaceId( this->spaces[_parent->GetName()] );
  link->SetWorld( _parent->GetWorld() );

  return link;
}

////////////////////////////////////////////////////////////////////////////////
// Create a new collision
CollisionPtr ODEPhysics::CreateCollision(const std::string &type, LinkPtr body)
{
  ODECollisionPtr collision( new ODECollision(body) );
  ShapePtr shape;

  if ( type == "sphere")
    shape.reset( new ODESphereShape(collision) );
  else if ( type == "plane")
    shape.reset( new ODEPlaneShape(collision) );
  else if ( type == "box")
    shape.reset( new ODEBoxShape(collision) );
  else if ( type == "cylinder")
    shape.reset( new ODECylinderShape(collision) );
  else if ( type == "multiray")
    shape.reset( new ODEMultiRayShape(collision) );
  else if ( type == "mesh" || type == "trimesh")
    shape.reset( new ODETrimeshShape(collision) );
  else if ( type == "heightmap")
    shape.reset( new ODEHeightmapShape(collision) );
  else if ( type == "map" || type == "image" )
    shape.reset( new MapShape(collision) );
  else
    gzerr << "Unable to create collision of type[" << type << "]\n";

  collision->SetShape(shape);
  shape->SetWorld(body->GetWorld());

  return collision;
}

////////////////////////////////////////////////////////////////////////////////
// Get the world id
dWorldID ODEPhysics::GetWorldId()
{
  return this->worldId;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert an odeMass to Mass
void ODEPhysics::ConvertMass(InertialPtr &_inertial, void *engineMass)
{
  dMass *odeMass = (dMass*)engineMass;

  _inertial->SetMass(odeMass->mass);
  _inertial->SetCoG( odeMass->c[0], odeMass->c[1], odeMass->c[2] );
  _inertial->SetInertiaMatrix( odeMass->I[0*4+0], odeMass->I[1*4+1],
      odeMass->I[2*4+2], odeMass->I[0*4+1],
      odeMass->I[0*4+2], odeMass->I[1*4+2] );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step iterations
void ODEPhysics::SetSORPGSIters(unsigned int iters)
{
  this->sdf->GetElement("ode")->GetElement("solver")->GetAttribute("iters")->Set(iters);
  dWorldSetQuickStepNumIterations(this->worldId, iters );
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetSORPGSW(double w)
{
  this->sdf->GetElement("ode")->GetElement("solver")->GetAttribute("sor")->Set(w);
  dWorldSetQuickStepW(this->worldId, w );
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetWorldCFM(double cfm)
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem =elem->GetOrCreateElement("constraints");
  elem->GetAttribute("cfm")->Set(cfm);

  dWorldSetCFM(this->worldId, cfm );
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetWorldERP(double erp)
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetOrCreateElement("constraints");
  elem->GetAttribute("erp")->Set(erp);
  dWorldSetERP(this->worldId, erp);
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetContactMaxCorrectingVel(double _vel)
{
  this->sdf->GetElement("ode")->GetOrCreateElement("constraints")->GetAttribute("contact_max_correcting_vel")->Set(_vel);
  dWorldSetContactMaxCorrectingVel(this->worldId, _vel);
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetContactSurfaceLayer(double _depth)
{
  this->sdf->GetElement("ode")->GetOrCreateElement("constraints")->GetAttribute("contact_surface_layer")->Set(_depth);
  dWorldSetContactSurfaceLayer(this->worldId, _depth);
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::SetMaxContacts(unsigned int _maxContacts)
{
  this->sdf->GetElement("ode")->GetOrCreateElement("max_contacts")->GetValue()->Set(_maxContacts);

  if (this->contactCollisions)
    delete this->contactCollisions;
  
  this->contactCollisions = new dContactGeom[_maxContacts];
}

////////////////////////////////////////////////////////////////////////////////
int ODEPhysics::GetSORPGSIters()
{
  return this->sdf->GetElement("ode")->GetElement("solver")->GetValueInt("iters");
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetSORPGSW()
{
  return this->sdf->GetElement("ode")->GetElement("solver")->GetValueDouble("sor");
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetWorldCFM()
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetOrCreateElement("constraints");
  return elem->GetValueDouble("cfm");
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetWorldERP()
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetOrCreateElement("constraints");
  return elem->GetValueDouble("erp");
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetContactMaxCorrectingVel()
{
  return this->sdf->GetElement("ode")->GetOrCreateElement("constraints")->GetValueDouble("contact_max_correcting_vel");
}

////////////////////////////////////////////////////////////////////////////////
double ODEPhysics::GetContactSurfaceLayer()
{
  return this->sdf->GetElement("ode")->GetOrCreateElement("constraints")->GetValueDouble("contact_surface_layer");
}

////////////////////////////////////////////////////////////////////////////////
int ODEPhysics::GetMaxContacts()
{
  return this->sdf->GetOrCreateElement("max_contacts")->GetValueInt();
}

////////////////////////////////////////////////////////////////////////////////
/// Convert an odeMass to Mass
void ODEPhysics::ConvertMass(void *engineMass, const InertialPtr &_inertial)
{
  dMass *odeMass = (dMass*)(engineMass);

  odeMass->mass = _inertial->GetMass();
  odeMass->c[0] = _inertial->GetCoG()[0];
  odeMass->c[1] = _inertial->GetCoG()[1];
  odeMass->c[2] = _inertial->GetCoG()[2];

  odeMass->I[0*4+0] = _inertial->GetPrincipalMoments()[0];
  odeMass->I[1*4+1] = _inertial->GetPrincipalMoments()[1];
  odeMass->I[2*4+2] = _inertial->GetPrincipalMoments()[2];

  odeMass->I[0*4+1] = _inertial->GetProductsofInertia()[0];
  odeMass->I[0*4+2] = _inertial->GetProductsofInertia()[1];
  odeMass->I[1*4+2] = _inertial->GetProductsofInertia()[2];
}

////////////////////////////////////////////////////////////////////////////////
// Create a new joint
JointPtr ODEPhysics::CreateJoint(const std::string &type)
{
  JointPtr joint;

  if (type == "prismatic")
    joint.reset( new ODESliderJoint(this->worldId) );
  if (type == "screw")
    joint.reset( new ODEScrewJoint(this->worldId) );
  if (type == "revolute")
    joint.reset( new ODEHingeJoint(this->worldId) );
  if (type == "revolute2")
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
  sdf::ElementPtr elem = this->sdf->GetElement("ode")->GetElement("solver");
  return elem->GetValueString("type");
}

////////////////////////////////////////////////////////////////////////////////
/// Set the step type
void ODEPhysics::SetStepType(const std::string type)
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode")->GetElement("solver");
  elem->GetAttribute("type")->Set(type);
  this->stepType = type;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gavity vector
void ODEPhysics::SetGravity(const gazebo::math::Vector3 &gravity)
{
  this->sdf->GetOrCreateElement("gravity")->GetAttribute("xyz")->Set(gravity);
  dWorldSetGravity(this->worldId, gravity.x, gravity.y, gravity.z);
}

////////////////////////////////////////////////////////////////////////////////
// Get gravity vector
math::Vector3 ODEPhysics::GetGravity() const
{
  return this->sdf->GetOrCreateElement("gravity")->GetValueVector3("xyz");
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
    ODECollision *collision1 = NULL;
    ODECollision *collision2 = NULL;

    // Exit if both bodies are not enabled
    if ( (b1 && b2 && !dBodyIsEnabled(b1) && !dBodyIsEnabled(b2)) || 
         (!b2 && b1 && !dBodyIsEnabled(b1)) || 
         (!b1 && b2 && !dBodyIsEnabled(b2)) )
    {
      return;
    }

    // Get pointers to the underlying collisions
    if (dGeomGetClass(o1) == dGeomTransformClass)
      collision1 = (ODECollision*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      collision1 = (ODECollision*) dGeomGetData(o1);

    if (dGeomGetClass(o2) == dGeomTransformClass)
      collision2 = (ODECollision*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      collision2 = (ODECollision*) dGeomGetData(o2);

    if (collision1->GetShapeType() == Base::TRIMESH_SHAPE ||
        collision2->GetShapeType() == Base::TRIMESH_SHAPE )
      self->AddTrimeshCollider(collision1, collision2);
    else
      self->AddCollider(collision1, collision2);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Collide two collisions
void ODEPhysics::Collide(ODECollision *collision1, ODECollision *collision2, 
                         dContactGeom *contactCollisions)
{
  int numContacts = 10;
  int j;
  int numc = 0;
  dContact contact;

  // for now, only use maxContacts if both collisionetries are trimeshes
  // other types of collisionetries do not need too many contacts
  if (collision1->GetShapeType() == Base::TRIMESH_SHAPE && 
      collision2->GetShapeType() == Base::TRIMESH_SHAPE)
  {
    numContacts = this->GetMaxContacts();
  }

  {
    tbb::spin_mutex::scoped_lock lock(this->collideMutex);
    numc = dCollide(collision1->GetCollisionId(), collision2->GetCollisionId(), 
        numContacts, contactCollisions, sizeof(contactCollisions[0]) );
  }

  if (numc != 0)
  {
    ContactFeedback *contactFeedback = NULL;

    // FIXME: this happens on a different thread then UpdateCollision?
    if (collision1->GetContactsEnabled() || collision2->GetContactsEnabled())
    {
      contactFeedback = new ContactFeedback();
      contactFeedback->contact.collision1 = collision1;
      contactFeedback->contact.collision2 = collision2;
      contactFeedback->feedbacks.resize(numc);
      this->contactFeedbacks.push_back(contactFeedback);
    }

    math::Vector3 contactPos;
    math::Vector3 contactNorm;
    double h, kp, kd;
    dJointID c;

    h = this->stepTimeDouble;
    for (j=0; j<numc; j++)
    {
      // skip negative depth contacts
      if(contactCollisions[j].depth < 0)
        continue;

      contact.geom = contactCollisions[j];

      contact.surface.mode = //dContactFDir1 | 
                             dContactBounce |
                             dContactMu2 |
                             dContactSoftERP | 
                             dContactSoftCFM | 
                             dContactSlip1 | 
                             dContactSlip2 | 
                             dContactApprox1;

      // TODO: put this back in.
      // Compute the CFM and ERP by assuming the two bodies form a
      // spring-damper system.
      kp = 1.0 / 
        (1.0 / collision1->surface->kp + 1.0 / collision2->surface->kp);
      kd = collision1->surface->kd + collision2->surface->kd;
      contact.surface.soft_erp = h * kp / (h * kp + kd);
      contact.surface.soft_cfm = 1.0 / (h * kp + kd);
      //contact.surface.soft_erp = 0.5*(collision1->surface->softERP +
      //                                collision2->surface->softERP);
      //contact.surface.soft_cfm = 0.5*(collision1->surface->softCFM +
      //                                collision2->surface->softCFM);

      //contact.fdir1[0] = 0.5*
      //(collision1->surface->fdir1.x+collision2->surface->fdir1.x);
      //contact.fdir1[1] = 0.5*
      //(collision1->surface->fdir1.y+collision2->surface->fdir1.y);
      //contact.fdir1[2] = 0.5*
      //(collision1->surface->fdir1.z+collision2->surface->fdir1.z);

      contact.surface.mu = std::min(collision1->surface->mu1, 
                                    collision2->surface->mu1);
      contact.surface.mu2 = std::min(collision1->surface->mu2, 
                                     collision2->surface->mu2);

      contact.surface.slip1 = std::min(collision1->surface->slip1, 
                                       collision2->surface->slip1);
      contact.surface.slip2 = std::min(collision1->surface->slip2, 
                                       collision2->surface->slip2);

      contact.surface.bounce = std::min(collision1->surface->bounce, 
                                        collision2->surface->bounce);
      contact.surface.bounce_vel = 
        std::min(collision1->surface->bounceThreshold, 
                 collision2->surface->bounceThreshold);

      {
        tbb::spin_mutex::scoped_lock lock(this->collideMutex);
        c = dJointCreateContact (this->worldId, this->contactGroup, &contact);

        contactPos.Set(contact.geom.pos[0], contact.geom.pos[1], 
                       contact.geom.pos[2]);
        contactNorm.Set(contact.geom.normal[0], contact.geom.normal[1], 
                        contact.geom.normal[2]);

        //this->AddContactVisual(contactPos, contactNorm);
      }

      // Store the contact info 
      if (contactFeedback)
      {
        contactFeedback->contact.depths.push_back(contact.geom.depth);
        contactFeedback->contact.positions.push_back(contactPos);
        contactFeedback->contact.normals.push_back(contactNorm);
        contactFeedback->contact.time = this->world->GetSimTime();
        dJointSetFeedback(c, &(contactFeedback->feedbacks[j]));
      }

      dBodyID b1 = dGeomGetBody(collision1->GetCollisionId());
      dBodyID b2 = dGeomGetBody(collision2->GetCollisionId());

      dJointAttach (c, b1, b2);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ODEPhysics::ProcessContactFeedback(ContactFeedback *_feedback)
{
  std::vector<dJointFeedback>::iterator jiter;

  if (_feedback->contact.collision1 == NULL)
    gzerr << "collision update Collision1 is null\n";

  if (_feedback->contact.collision2 == NULL)
    gzerr << "Collision update Collision2 is null\n";

  _feedback->contact.forces.clear();

  // Copy all the joint forces to the contact
  for (jiter = _feedback->feedbacks.begin(); 
       jiter != _feedback->feedbacks.end(); jiter++)
  {
    JointFeedback joint;
    joint.body1Force.Set((*jiter).f1[0], (*jiter).f1[1], (*jiter).f1[2]);
    joint.body2Force.Set((*jiter).f2[0], (*jiter).f2[1], (*jiter).f2[2]);

    joint.body1Torque.Set((*jiter).t1[0], (*jiter).t1[1], (*jiter).t1[2]);
    joint.body2Torque.Set((*jiter).t2[0], (*jiter).t2[1], (*jiter).t2[2]);

    _feedback->contact.forces.push_back(joint);
  }

  // Add the contact to each collision
  _feedback->contact.collision1->AddContact( _feedback->contact );
  _feedback->contact.collision2->AddContact( _feedback->contact );
}

void ODEPhysics::AddTrimeshCollider( ODECollision *_collision1, 
                                     ODECollision *_collision2 )
{
  if (this->trimeshCollidersCount >= this->trimeshColliders.size())
    this->trimeshColliders.resize(this->trimeshColliders.size() + 100);

  this->trimeshColliders[this->trimeshCollidersCount].first  = _collision1;
  this->trimeshColliders[this->trimeshCollidersCount].second = _collision2;
  this->trimeshCollidersCount++;
}

void ODEPhysics::AddCollider( ODECollision *_collision1, 
                              ODECollision *_collision2 )
{
  if (this->collidersCount >= this->colliders.size())
    this->colliders.resize(this->colliders.size() + 100);

  this->colliders[this->collidersCount].first  = _collision1;
  this->colliders[this->collidersCount].second = _collision2;
  this->collidersCount++;
}
