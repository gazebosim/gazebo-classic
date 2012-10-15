/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo_config.h"
#include "common/Diagnostics.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"
#include "common/Time.hh"
#include "common/Timer.hh"

#include "transport/Publisher.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/Model.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Collision.hh"
#include "physics/MapShape.hh"

#include "physics/ode/ODECollision.hh"
#include "physics/ode/ODELink.hh"
#include "physics/ode/ODEScrewJoint.hh"
#include "physics/ode/ODEHingeJoint.hh"
#include "physics/ode/ODEHinge2Joint.hh"
#include "physics/ode/ODESliderJoint.hh"
#include "physics/ode/ODEBallJoint.hh"
#include "physics/ode/ODEUniversalJoint.hh"

#include "physics/ode/ODERayShape.hh"
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
  public: ContactUpdate_TBB(tbb::concurrent_vector<ContactFeedback> *_contacts)
          : contacts(contacts) {}
  public: void operator() (const tbb::blocked_range<size_t> &_r) const
  {
    for (size_t i = _r.begin(); i != _r.end(); i++)
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
              std::vector<std::pair<ODECollision*, ODECollision*> > *_colliders,
              ODEPhysics *_engine,
              dContactGeom* _contactCollisions) :
    colliders(_colliders),
              engine(_engine), contactCollisions(_contactCollisions)
  {
    // dAllocateODEDataForThread(dAllocateMaskAll);
  }

  public: void operator() (const tbb::blocked_range<size_t> &_r) const
  {
    for (size_t i = _r.begin(); i != _r.end(); i++)
    {
      ODECollision *collision1 = (*this->colliders)[i].first;
      ODECollision *collision2 = (*this->colliders)[i].second;
      this->engine->Collide(collision1, collision2, contactCollisions);
    }
  }

  private: std::vector< std::pair<ODECollision*, ODECollision*> > *colliders;
  private: ODEPhysics *engine;
  private: dContactGeom* contactCollisions;
};

//////////////////////////////////////////////////
ODEPhysics::ODEPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  // Collision detection init
  dInitODE2(0);

  dAllocateODEDataForThread(dAllocateMaskAll);

  this->worldId = dWorldCreate();

  this->spaceId = dHashSpaceCreate(0);
  dHashSpaceSetLevels(this->spaceId, -2, 8);

  this->contactGroup = dJointGroupCreate(0);

  this->contactFeedbackIndex = 0;
  this->colliders.resize(100);
}

//////////////////////////////////////////////////
ODEPhysics::~ODEPhysics()
{
  dCloseODE();

  for (unsigned int i = 0; i < this->contactFeedbacks.size(); i++)
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
}

//////////////////////////////////////////////////
void ODEPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  sdf::ElementPtr odeElem = this->sdf->GetElement("ode");
  sdf::ElementPtr solverElem = odeElem->GetElement("solver");

  this->stepTimeDouble = solverElem->GetValueDouble("dt");
  this->stepType = solverElem->GetValueString("type");

  dWorldSetDamping(this->worldId, 0.0001, 0.0001);

  // Help prevent "popping of deeply embedded object
  dWorldSetContactMaxCorrectingVel(this->worldId,
      odeElem->GetElement("constraints")->GetValueDouble(
        "contact_max_correcting_vel"));

  // This helps prevent jittering problems.
  dWorldSetContactSurfaceLayer(this->worldId,
       odeElem->GetElement("constraints")->GetValueDouble(
        "contact_surface_layer"));

  // If auto-disable is active, then user interaction with the joints
  // doesn't behave properly
  // disable autodisable by default
  dWorldSetAutoDisableFlag(this->worldId, 0);

  dWorldSetAutoDisableTime(this->worldId, 2);
  dWorldSetAutoDisableLinearThreshold(this->worldId, 0.01);
  dWorldSetAutoDisableAngularThreshold(this->worldId, 0.01);
  dWorldSetAutoDisableSteps(this->worldId, 20);

  math::Vector3 g = this->sdf->GetValueVector3("gravity");

  if (g == math::Vector3(0, 0, 0))
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";

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

  dWorldSetQuickStepNumIterations(this->worldId, this->GetSORPGSIters());
  dWorldSetQuickStepW(this->worldId, this->GetSORPGSW());

  // Set the physics update function
  if (this->stepType == "quick")
    this->physicsStepFunc = &dWorldQuickStep;
  else if (this->stepType == "world")
    this->physicsStepFunc = &dWorldStep;
  else
    gzthrow(std::string("Invalid step type[") + this->stepType);
}

/////////////////////////////////////////////////
void ODEPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::ODE);
    physicsMsg.set_update_rate(this->GetUpdateRate());
    physicsMsg.set_solver_type(this->stepType);
    physicsMsg.set_dt(this->stepTimeDouble);
    physicsMsg.set_iters(this->GetSORPGSIters());
    physicsMsg.set_sor(this->GetSORPGSW());
    physicsMsg.set_cfm(this->GetWorldCFM());
    physicsMsg.set_erp(this->GetWorldERP());
    physicsMsg.set_contact_max_correcting_vel(
        this->GetContactMaxCorrectingVel());
    physicsMsg.set_contact_surface_layer(this->GetContactSurfaceLayer());
    physicsMsg.mutable_gravity()->CopyFrom(msgs::Convert(this->GetGravity()));

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

/////////////////////////////////////////////////
void ODEPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  if (_msg->has_dt())
  {
    this->SetStepTime(_msg->dt());
  }

  if (_msg->has_update_rate())
    this->SetUpdateRate(_msg->update_rate());

  if (_msg->has_solver_type())
  {
    sdf::ElementPtr solverElem =
      this->sdf->GetElement("ode")->GetElement("solver");
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
    this->SetWorldCFM(_msg->cfm());

  if (_msg->has_erp())
    this->SetWorldERP(_msg->erp());

  if (_msg->has_contact_max_correcting_vel())
    this->SetContactMaxCorrectingVel(_msg->contact_max_correcting_vel());

  if (_msg->has_contact_surface_layer())
    this->SetContactSurfaceLayer(_msg->contact_surface_layer());

  if (_msg->has_gravity())
    this->SetGravity(msgs::Convert(_msg->gravity()));

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}



//////////////////////////////////////////////////
void ODEPhysics::Init()
{
}

//////////////////////////////////////////////////
void ODEPhysics::InitForThread()
{
  dAllocateODEDataForThread(dAllocateMaskAll);
}

//////////////////////////////////////////////////
void ODEPhysics::UpdateCollision()
{
  unsigned int i = 0;
  this->collidersCount = 0;
  this->trimeshCollidersCount = 0;

  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide(this->spaceId, this, CollisionCallback);

  this->contactFeedbackIndex = 0;

  for (i = 0; i < this->collidersCount; ++i)
  {
    this->Collide(this->colliders[i].first,
        this->colliders[i].second, this->contactCollisions);
  }

  // Trimesh collision must happen in this thread sequentially
  for (i = 0; i < this->trimeshCollidersCount; ++i)
  {
    ODECollision *collision1 = this->trimeshColliders[i].first;
    ODECollision *collision2 = this->trimeshColliders[i].second;
    this->Collide(collision1, collision2, this->contactCollisions);
  }
}

//////////////////////////////////////////////////
void ODEPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  {
    this->physicsUpdateMutex->lock();

    // Update the dynamical model
    (*physicsStepFunc)(this->worldId, this->stepTimeDouble);

    msgs::Contacts msg;

    // put contact forces into contact feedbacks
    for (unsigned int i = 0; i < this->contactFeedbackIndex; ++i)
      this->ProcessContactFeedback(this->contactFeedbacks[i],
                                   msg.add_contact());

    this->contactPub->Publish(msg);

    dJointGroupEmpty(this->contactGroup);
    this->physicsUpdateMutex->unlock();
  }
}

//////////////////////////////////////////////////
void ODEPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void ODEPhysics::Reset()
{
  this->physicsUpdateMutex->lock();
  // Very important to clear out the contact group
  dJointGroupEmpty(this->contactGroup);
  this->physicsUpdateMutex->unlock();
}



//////////////////////////////////////////////////
void ODEPhysics::SetStepTime(double _value)
{
  this->sdf->GetElement("ode")->GetElement(
      "solver")->GetElement("dt")->Set(_value);

  this->stepTimeDouble = _value;
}

//////////////////////////////////////////////////
double ODEPhysics::GetStepTime()
{
  return this->stepTimeDouble;
}

//////////////////////////////////////////////////
LinkPtr ODEPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  std::map<std::string, dSpaceID>::iterator iter;
  iter = this->spaces.find(_parent->GetName());

  if (iter == this->spaces.end())
    this->spaces[_parent->GetName()] = dSimpleSpaceCreate(this->spaceId);

  ODELinkPtr link(new ODELink(_parent));

  link->SetSpaceId(this->spaces[_parent->GetName()]);
  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr ODEPhysics::CreateCollision(const std::string &_type,
                                         LinkPtr _body)
{
  ODECollisionPtr collision(new ODECollision(_body));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_body->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr ODEPhysics::CreateShape(const std::string &_type,
                                 CollisionPtr _collision)
{
  ShapePtr shape;
  ODECollisionPtr collision =
    boost::shared_dynamic_cast<ODECollision>(_collision);

  if (_type == "sphere")
    shape.reset(new ODESphereShape(collision));
  else if (_type == "plane")
    shape.reset(new ODEPlaneShape(collision));
  else if (_type == "box")
    shape.reset(new ODEBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new ODECylinderShape(collision));
  else if (_type == "multiray")
    shape.reset(new ODEMultiRayShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new ODETrimeshShape(collision));
  else if (_type == "heightmap")
    shape.reset(new ODEHeightmapShape(collision));
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new ODERayShape(collision));
    else
      shape.reset(new ODERayShape(this->world->GetPhysicsEngine()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  return shape;
}

//////////////////////////////////////////////////
dWorldID ODEPhysics::GetWorldId()
{
  return this->worldId;
}

//////////////////////////////////////////////////
void ODEPhysics::ConvertMass(InertialPtr _inertial, void *_engineMass)
{
  dMass *odeMass = static_cast<dMass*>(_engineMass);

  _inertial->SetMass(odeMass->mass);
  _inertial->SetCoG(odeMass->c[0], odeMass->c[1], odeMass->c[2]);
  _inertial->SetInertiaMatrix(odeMass->I[0*4+0], odeMass->I[1*4+1],
      odeMass->I[2*4+2], odeMass->I[0*4+1],
      odeMass->I[0*4+2], odeMass->I[1*4+2]);
}

//////////////////////////////////////////////////
void ODEPhysics::SetSORPGSPreconIters(unsigned int _iters)
{
  this->sdf->GetElement("ode")->GetElement("solver")->
    GetElement("percon_iters")->Set(_iters);

  dWorldSetQuickStepPreconIterations(this->worldId, _iters);
}

//////////////////////////////////////////////////
void ODEPhysics::SetSORPGSIters(unsigned int _iters)
{
  this->sdf->GetElement("ode")->GetElement(
      "solver")->GetElement("iters")->Set(_iters);
  dWorldSetQuickStepNumIterations(this->worldId, _iters);
}

//////////////////////////////////////////////////
void ODEPhysics::SetSORPGSW(double _w)
{
  this->sdf->GetElement("ode")->GetElement(
      "solver")->GetElement("sor")->Set(_w);
  dWorldSetQuickStepW(this->worldId, _w);
}

//////////////////////////////////////////////////
void ODEPhysics::SetWorldCFM(double _cfm)
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetElement("constraints");
  elem->GetElement("cfm")->Set(_cfm);

  dWorldSetCFM(this->worldId, _cfm);
}

//////////////////////////////////////////////////
void ODEPhysics::SetWorldERP(double _erp)
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetElement("constraints");
  elem->GetElement("erp")->Set(_erp);
  dWorldSetERP(this->worldId, _erp);
}

//////////////////////////////////////////////////
void ODEPhysics::SetContactMaxCorrectingVel(double _vel)
{
  this->sdf->GetElement("ode")->GetElement(
      "constraints")->GetElement(
        "contact_max_correcting_vel")->Set(_vel);
  dWorldSetContactMaxCorrectingVel(this->worldId, _vel);
}

//////////////////////////////////////////////////
void ODEPhysics::SetContactSurfaceLayer(double _depth)
{
  this->sdf->GetElement("ode")->GetElement(
      "constraints")->GetElement("contact_surface_layer")->Set(_depth);
  dWorldSetContactSurfaceLayer(this->worldId, _depth);
}

//////////////////////////////////////////////////
void ODEPhysics::SetMaxContacts(unsigned int _maxContacts)
{
  this->sdf->GetElement("ode")->GetElement(
      "max_contacts")->GetValue()->Set(_maxContacts);
}

//////////////////////////////////////////////////
int ODEPhysics::GetSORPGSPreconIters()
{
  return this->sdf->GetElement("ode")->GetElement(
      "solver")->GetValueInt("precon_iters");
}
//////////////////////////////////////////////////
int ODEPhysics::GetSORPGSIters()
{
  return this->sdf->GetElement("ode")->GetElement(
      "solver")->GetValueInt("iters");
}

//////////////////////////////////////////////////
double ODEPhysics::GetSORPGSW()
{
  return this->sdf->GetElement("ode")->GetElement(
      "solver")->GetValueDouble("sor");
}

//////////////////////////////////////////////////
double ODEPhysics::GetWorldCFM()
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetElement("constraints");
  return elem->GetValueDouble("cfm");
}

//////////////////////////////////////////////////
double ODEPhysics::GetWorldERP()
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode");
  elem = elem->GetElement("constraints");
  return elem->GetValueDouble("erp");
}

//////////////////////////////////////////////////
double ODEPhysics::GetContactMaxCorrectingVel()
{
  return this->sdf->GetElement("ode")->GetElement(
      "constraints")->GetValueDouble("contact_max_correcting_vel");
}

//////////////////////////////////////////////////
double ODEPhysics::GetContactSurfaceLayer()
{
  return this->sdf->GetElement("ode")->GetElement(
      "constraints")->GetValueDouble("contact_surface_layer");
}

//////////////////////////////////////////////////
int ODEPhysics::GetMaxContacts()
{
  return this->sdf->GetElement("max_contacts")->GetValueInt();
}

//////////////////////////////////////////////////
void ODEPhysics::ConvertMass(void *_engineMass, InertialPtr _inertial)
{
  dMass *odeMass = static_cast<dMass*>(_engineMass);

  odeMass->mass = _inertial->GetMass();
  odeMass->c[0] = _inertial->GetCoG()[0];
  odeMass->c[1] = _inertial->GetCoG()[1];
  odeMass->c[2] = _inertial->GetCoG()[2];

  odeMass->I[0*4+0] = _inertial->GetPrincipalMoments()[0];
  odeMass->I[1*4+1] = _inertial->GetPrincipalMoments()[1];
  odeMass->I[2*4+2] = _inertial->GetPrincipalMoments()[2];

  odeMass->I[0*4+1] = _inertial->GetProductsofInertia()[0];
  odeMass->I[1*4+0] = _inertial->GetProductsofInertia()[0];

  odeMass->I[0*4+2] = _inertial->GetProductsofInertia()[1];
  odeMass->I[1*4+0] = _inertial->GetProductsofInertia()[1];

  odeMass->I[1*4+2] = _inertial->GetProductsofInertia()[2];
  odeMass->I[2*4+1] = _inertial->GetProductsofInertia()[2];
}

//////////////////////////////////////////////////
JointPtr ODEPhysics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

  if (_type == "prismatic")
    joint.reset(new ODESliderJoint(this->worldId, _parent));
  else if (_type == "screw")
    joint.reset(new ODEScrewJoint(this->worldId, _parent));
  else if (_type == "revolute")
    joint.reset(new ODEHingeJoint(this->worldId, _parent));
  else if (_type == "revolute2")
    joint.reset(new ODEHinge2Joint(this->worldId, _parent));
  else if (_type == "ball")
    joint.reset(new ODEBallJoint(this->worldId, _parent));
  else if (_type == "universal")
    joint.reset(new ODEUniversalJoint(this->worldId, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
dSpaceID ODEPhysics::GetSpaceId() const
{
  return this->spaceId;
}

//////////////////////////////////////////////////
std::string ODEPhysics::GetStepType() const
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode")->GetElement("solver");
  return elem->GetValueString("type");
}

//////////////////////////////////////////////////
void ODEPhysics::SetStepType(const std::string &_type)
{
  sdf::ElementPtr elem = this->sdf->GetElement("ode")->GetElement("solver");
  elem->GetElement("type")->Set(_type);
  this->stepType = _type;
}

//////////////////////////////////////////////////
void ODEPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->Set(_gravity);
  dWorldSetGravity(this->worldId, _gravity.x, _gravity.y, _gravity.z);
}

//////////////////////////////////////////////////
math::Vector3 ODEPhysics::GetGravity() const
{
  return this->sdf->GetValueVector3("gravity");
}

//////////////////////////////////////////////////
void ODEPhysics::CollisionCallback(void *_data, dGeomID _o1, dGeomID _o2)
{
  dBodyID b1 = dGeomGetBody(_o1);
  dBodyID b2 = dGeomGetBody(_o2);

  // exit without doing anything if the two bodies are connected by a joint
  if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
    return;

  ODEPhysics *self = static_cast<ODEPhysics*>(_data);

  // Check if either are spaces
  if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
  {
    dSpaceCollide2(_o1, _o2, self, &CollisionCallback);
  }
  else
  {
    ODECollision *collision1 = NULL;
    ODECollision *collision2 = NULL;

    // Exit if both bodies are not enabled
    if ((b1 && b2 && !dBodyIsEnabled(b1) && !dBodyIsEnabled(b2)) ||
         (!b2 && b1 && !dBodyIsEnabled(b1)) ||
         (!b1 && b2 && !dBodyIsEnabled(b2)))
    {
      return;
    }

    // Get pointers to the underlying collisions
    if (dGeomGetClass(_o1) == dGeomTransformClass)
      collision1 =
        static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o1)));
    else
      collision1 = static_cast<ODECollision*>(dGeomGetData(_o1));

    if (dGeomGetClass(_o2) == dGeomTransformClass)
      collision2 =
        static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
    else
      collision2 = static_cast<ODECollision*>(dGeomGetData(_o2));

    if (collision1 && collision2)
    {
      // BUG: == is not right, should use &
      if (collision1->HasType(Base::TRIMESH_SHAPE) ||
          collision2->HasType(Base::TRIMESH_SHAPE))
        self->AddTrimeshCollider(collision1, collision2);
      else
      {
        self->AddCollider(collision1, collision2);
      }
    }
  }
}


//////////////////////////////////////////////////
void ODEPhysics::Collide(ODECollision *_collision1, ODECollision *_collision2,
                         dContactGeom *_contactCollisions)
{
  int numc = 0;
  dContact contact;

  // maxCollide must less than the size of this->indices. Check the header
  int maxCollide = MAX_CONTACT_JOINTS;
  if (this->GetMaxContacts() < MAX_CONTACT_JOINTS)
    maxCollide = this->GetMaxContacts();

  numc = dCollide(_collision1->GetCollisionId(), _collision2->GetCollisionId(),
      MAX_COLLIDE_RETURNS, _contactCollisions, sizeof(_contactCollisions[0]));

  if (numc <= 0)
    return;

  for (int i = 0; i < MAX_CONTACT_JOINTS; i++)
    this->indices[i] = i;

  if (numc > maxCollide)
  {
    double max = _contactCollisions[maxCollide-1].depth;
    for (int i = maxCollide; i < numc; i++)
    {
      if (_contactCollisions[i].depth > max)
      {
        max = _contactCollisions[i].depth;
        this->indices[maxCollide-1] = i;
      }
    }
    numc = maxCollide;
  }

  ContactFeedback *contactFeedback = NULL;

  if (_collision1->GetContactsEnabled() || _collision2->GetContactsEnabled())
  {
    if (this->contactFeedbackIndex < this->contactFeedbacks.size())
      contactFeedback = this->contactFeedbacks[this->contactFeedbackIndex++];
    else
    {
      contactFeedback = new ContactFeedback();
      this->contactFeedbacks.push_back(contactFeedback);
      this->contactFeedbackIndex = this->contactFeedbacks.size();
    }

    contactFeedback->contact.collision1 = _collision1;
    contactFeedback->contact.collision2 = _collision2;
    contactFeedback->feedbackCount = 0;
    contactFeedback->contact.count = 0;
    contactFeedback->contact.time = this->world->GetSimTime();
  }

  double h = this->stepTimeDouble;

  contact.surface.mode = dContactBounce |
                         dContactMu2 |
                         dContactSoftERP |
                         dContactSoftCFM |
                         dContactApprox1 |
                         dContactSlip1 |
                         dContactSlip2;

  // Compute the CFM and ERP by assuming the two bodies form a
  // spring-damper system.
  double kp = 1.0 /
    (1.0 / _collision1->GetSurface()->kp + 1.0 / _collision2->GetSurface()->kp);
  double kd = _collision1->GetSurface()->kd + _collision2->GetSurface()->kd;

  contact.surface.soft_erp = h * kp / (h * kp + kd);
  contact.surface.soft_cfm = 1.0 / (h * kp + kd);

  // contact.surface.soft_erp = 0.5*(_collision1->surface->softERP +
  //                                _collision2->surface->softERP);
  // contact.surface.soft_cfm = 0.5*(_collision1->surface->softCFM +
  //                                _collision2->surface->softCFM);

  // assign fdir1 if not set as 0
  math::Vector3 fd =
    (_collision1->GetSurface()->fdir1 +_collision2->GetSurface()->fdir1) * 0.5;
  if (fd != math::Vector3(0, 0, 0))
  {
    contact.surface.mode |= dContactFDir1;
    contact.fdir1[0] = fd.x;
    contact.fdir1[1] = fd.y;
    contact.fdir1[2] = fd.z;
  }

  contact.surface.mu = std::min(_collision1->GetSurface()->mu1,
                                _collision2->GetSurface()->mu1);
  contact.surface.mu2 = std::min(_collision1->GetSurface()->mu2,
                                 _collision2->GetSurface()->mu2);


  contact.surface.slip1 = std::min(_collision1->GetSurface()->slip1,
                                   _collision2->GetSurface()->slip1);
  contact.surface.slip2 = std::min(_collision1->GetSurface()->slip2,
                                   _collision2->GetSurface()->slip2);

  contact.surface.bounce = std::min(_collision1->GetSurface()->bounce,
                                    _collision2->GetSurface()->bounce);
  contact.surface.bounce_vel =
    std::min(_collision1->GetSurface()->bounceThreshold,
             _collision2->GetSurface()->bounceThreshold);

  dBodyID b1 = dGeomGetBody(_collision1->GetCollisionId());
  dBodyID b2 = dGeomGetBody(_collision2->GetCollisionId());

  for (int j = 0; j < numc; j++)
  {
    // A depth of <0 may never occur. Commenting this out for now.
    // skip negative depth contacts
    if (_contactCollisions[this->indices[j]].depth < 0)
    {
      gzerr << "negative depth ["
            << _contactCollisions[this->indices[j]].depth << "]\n";
      continue;
    }

    contact.geom = _contactCollisions[this->indices[j]];

    dJointID contact_joint =
      dJointCreateContact(this->worldId, this->contactGroup, &contact);

    // Store the contact info
    if (contactFeedback)
    {
      contactFeedback->contact.depths[j] = contact.geom.depth;

      contactFeedback->contact.positions[j].Set(contact.geom.pos[0],
          contact.geom.pos[1], contact.geom.pos[2]);

      contactFeedback->contact.normals[j].Set(contact.geom.normal[0],
          contact.geom.normal[1], contact.geom.normal[2]);

      contactFeedback->contact.count++;
      contactFeedback->feedbackCount++;

      dJointSetFeedback(contact_joint, &(contactFeedback->feedbacks[j]));
    }

    dJointAttach(contact_joint, b1, b2);
  }
}

//////////////////////////////////////////////////
void ODEPhysics::ProcessContactFeedback(ContactFeedback *_feedback,
                                        msgs::Contact *_msg)
{
  if (_feedback->contact.collision1 == NULL)
    gzerr << "collision update Collision1 is null\n";

  if (_feedback->contact.collision2 == NULL)
    gzerr << "Collision update Collision2 is null\n";

  _msg->set_collision1(_feedback->contact.collision1->GetScopedName());
  _msg->set_collision2(_feedback->contact.collision2->GetScopedName());

  // Copy all the joint forces to the contact
  for (int i = 0; i < _feedback->feedbackCount; i++)
  {
    msgs::Set(_msg->add_position(), _feedback->contact.positions[i]);
    msgs::Set(_msg->add_normal(), _feedback->contact.normals[i]);
    _msg->add_depth(_feedback->contact.depths[i]);

    _feedback->contact.forces[i].body1Force.Set(
        _feedback->feedbacks[i].f1[0],
        _feedback->feedbacks[i].f1[1],
        _feedback->feedbacks[i].f1[2]);

    _feedback->contact.forces[i].body2Force.Set(
        _feedback->feedbacks[i].f2[0],
        _feedback->feedbacks[i].f2[1],
        _feedback->feedbacks[i].f2[2]);

    _feedback->contact.forces[i].body1Torque.Set(
        _feedback->feedbacks[i].t1[0],
        _feedback->feedbacks[i].t1[1],
        _feedback->feedbacks[i].t1[2]);

    _feedback->contact.forces[i].body2Torque.Set(
        _feedback->feedbacks[i].t2[0],
        _feedback->feedbacks[i].t2[1],
        _feedback->feedbacks[i].t2[2]);
  }

  // Add the contact to each collision
  _feedback->contact.collision1->AddContact(_feedback->contact);
  _feedback->contact.collision2->AddContact(_feedback->contact);
}

/////////////////////////////////////////////////
void ODEPhysics::AddTrimeshCollider(ODECollision *_collision1,
                                     ODECollision *_collision2)
{
  if (this->trimeshCollidersCount >= this->trimeshColliders.size())
    this->trimeshColliders.resize(this->trimeshColliders.size() + 100);

  this->trimeshColliders[this->trimeshCollidersCount].first  = _collision1;
  this->trimeshColliders[this->trimeshCollidersCount].second = _collision2;
  this->trimeshCollidersCount++;
}

/////////////////////////////////////////////////
void ODEPhysics::AddCollider(ODECollision *_collision1,
                             ODECollision *_collision2)
{
  if (this->collidersCount >= this->colliders.size())
    this->colliders.resize(this->colliders.size() + 100);

  this->colliders[this->collidersCount].first  = _collision1;
  this->colliders[this->collidersCount].second = _collision2;
  this->collidersCount++;
}

/////////////////////////////////////////////////
void ODEPhysics::DebugPrint() const
{
  dBodyID b;
  std::cout << "Debug Print[" << dWorldGetBodyCount(this->worldId) << "]\n";
  for (int i = 0; i < dWorldGetBodyCount(this->worldId); ++i)
  {
    b = dWorldGetBody(this->worldId, i);
    ODELink *link = static_cast<ODELink*>(dBodyGetData(b));
    math::Pose pose = link->GetWorldPose();
    const dReal *pos = dBodyGetPosition(b);
    const dReal *rot = dBodyGetRotation(b);
    math::Vector3 dpos(pos[0], pos[1], pos[2]);
    math::Quaternion drot(rot[0], rot[1], rot[2], rot[3]);

    std::cout << "Body[" << link->GetScopedName() << "]\n";
    std::cout << "  World: Pos[" << dpos << "] Rot[" << drot << "]\n";
    if (pose.pos != dpos)
      std::cout << "    Incorrect world pos[" << pose.pos << "]\n";
    if (pose.rot != drot)
      std::cout << "    Incorrect world rot[" << pose.rot << "]\n";

    dMass mass;
    dBodyGetMass(b, &mass);
    std::cout << "  Mass[" << mass.mass << "] COG[" << mass.c[0]
              << " " << mass.c[1] << " " << mass.c[2] << "]\n";

    dGeomID g = dBodyGetFirstGeom(b);
    while (g)
    {
      ODECollision *coll = static_cast<ODECollision*>(dGeomGetData(g));

      pose = coll->GetWorldPose();
      const dReal *gpos = dGeomGetPosition(g);
      const dReal *grot = dGeomGetRotation(g);
      dpos.Set(gpos[0], gpos[1], gpos[2]);
      drot.Set(grot[0], grot[1], grot[2], grot[3]);

      std::cout << "    Geom[" << coll->GetScopedName() << "]\n";
      std::cout << "      World: Pos[" << dpos << "] Rot[" << drot << "]\n";

      if (pose.pos != dpos)
        std::cout << "      Incorrect world pos[" << pose.pos << "]\n";
      if (pose.rot != drot)
        std::cout << "      Incorrect world rot[" << pose.rot << "]\n";

      g = dBodyGetNextGeom(g);
    }
  }
}
