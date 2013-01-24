/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Diagnostics.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/ContactManager.hh"

#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODEScrewJoint.hh"
#include "gazebo/physics/ode/ODEHingeJoint.hh"
#include "gazebo/physics/ode/ODEHinge2Joint.hh"
#include "gazebo/physics/ode/ODESliderJoint.hh"
#include "gazebo/physics/ode/ODEBallJoint.hh"
#include "gazebo/physics/ode/ODEUniversalJoint.hh"

#include "gazebo/physics/ode/ODERayShape.hh"
#include "gazebo/physics/ode/ODEBoxShape.hh"
#include "gazebo/physics/ode/ODESphereShape.hh"
#include "gazebo/physics/ode/ODECylinderShape.hh"
#include "gazebo/physics/ode/ODEPlaneShape.hh"
#include "gazebo/physics/ode/ODETrimeshShape.hh"
#include "gazebo/physics/ode/ODEMultiRayShape.hh"
#include "gazebo/physics/ode/ODEHeightmapShape.hh"

#include "gazebo/physics/ode/ODEPhysics.hh"

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

  this->colliders.resize(100);
}

//////////////////////////////////////////////////
ODEPhysics::~ODEPhysics()
{
  dCloseODE();

  dJointGroupDestroy(this->contactGroup);

  // Delete all the joint feedbacks.
  for (std::vector<ODEJointFeedback*>::iterator iter =
      this->jointFeedbacks.begin(); iter != this->jointFeedbacks.end(); ++iter)
  {
    delete *iter;
  }
  this->jointFeedbacks.clear();

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

  // Enable auto-disable by default. Models with joints are excluded from
  // auto-disable
  dWorldSetAutoDisableFlag(this->worldId, 1);

  dWorldSetAutoDisableTime(this->worldId, 1);
  dWorldSetAutoDisableLinearThreshold(this->worldId, 0.1);
  dWorldSetAutoDisableAngularThreshold(this->worldId, 0.1);
  dWorldSetAutoDisableSteps(this->worldId, 5);

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
    physicsMsg.set_enable_physics(this->world->GetEnablePhysicsEngine());
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

  if (_msg->has_enable_physics())
    this->world->EnablePhysicsEngine(_msg->enable_physics());

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
  {
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
    dJointGroupEmpty(this->contactGroup);
  }

  unsigned int i = 0;
  this->collidersCount = 0;
  this->trimeshCollidersCount = 0;
  this->jointFeedbackIndex = 0;

  // Reset the contact count
  this->contactManager->ResetCount();

  // Do collision detection; this will add contacts to the contact group
  dSpaceCollide(this->spaceId, this, CollisionCallback);

  // Generate non-trimesh collisions.
  for (i = 0; i < this->collidersCount; ++i)
  {
    this->Collide(this->colliders[i].first,
        this->colliders[i].second, this->contactCollisions);
  }

  // Generate trimesh collision.
  // This must happen in this thread sequentially
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
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

    // Update the dynamical model
    (*physicsStepFunc)(this->worldId, this->stepTimeDouble);

    // Set the joint contact feedback for each contact.
    for (unsigned int i = 0; i < this->jointFeedbackIndex; ++i)
    {
      for (int j = 0; j < this->jointFeedbacks[i]->count; ++j)
      {
        this->jointFeedbacks[i]->contact->wrench[j].body1Force.Set(
            this->jointFeedbacks[i]->feedbacks[j].f1[0],
            this->jointFeedbacks[i]->feedbacks[j].f1[1],
            this->jointFeedbacks[i]->feedbacks[j].f1[2]);

        this->jointFeedbacks[i]->contact->wrench[j].body2Force.Set(
            this->jointFeedbacks[i]->feedbacks[j].f2[0],
            this->jointFeedbacks[i]->feedbacks[j].f2[1],
            this->jointFeedbacks[i]->feedbacks[j].f2[2]);

        this->jointFeedbacks[i]->contact->wrench[j].body1Torque.Set(
            this->jointFeedbacks[i]->feedbacks[j].t1[0],
            this->jointFeedbacks[i]->feedbacks[j].t1[1],
            this->jointFeedbacks[i]->feedbacks[j].t1[2]);

        this->jointFeedbacks[i]->contact->wrench[j].body2Torque.Set(
            this->jointFeedbacks[i]->feedbacks[j].t2[0],
            this->jointFeedbacks[i]->feedbacks[j].t2[1],
            this->jointFeedbacks[i]->feedbacks[j].t2[2]);
      }
    }
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
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
  // Very important to clear out the contact group
  dJointGroupEmpty(this->contactGroup);
}

//////////////////////////////////////////////////
void ODEPhysics::SetStepTime(double _value)
{
  this->sdf->GetElement("ode")->GetElement(
      "solver")->GetElement("dt")->Set(_value);

  this->stepTimeDouble = _value;
}

//////////////////////////////////////////////////
double ODEPhysics::GetStepTime() const
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
void ODEPhysics::CollisionCallback(void *_data, dGeomID _o1, dGeomID _o2)
{
  dBodyID b1 = dGeomGetBody(_o1);
  dBodyID b2 = dGeomGetBody(_o2);

  // exit without doing anything if the two bodies are connected by a joint
  if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
    return;

  // Get a pointer to the physics engine
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

    // Make sure both collision pointers are valid.
    if (collision1 && collision2)
    {
      // Add either a tri-mesh collider or a regular collider.
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

  // Generate the contacts
  numc = dCollide(_collision1->GetCollisionId(), _collision2->GetCollisionId(),
      MAX_COLLIDE_RETURNS, _contactCollisions, sizeof(_contactCollisions[0]));

  // Return if no contacts.
  if (numc <= 0)
    return;

  // Store the indices of the contacts.
  for (int i = 0; i < MAX_CONTACT_JOINTS; i++)
    this->indices[i] = i;

  // Choose only the best contacts if too many were generated.
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

    // Make sure numc has the valid number of contacts.
    numc = maxCollide;
  }

  // Set the contact surface parameter flags.
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

  contact.surface.soft_erp = (this->stepTimeDouble * kp) /
                             (this->stepTimeDouble * kp + kd);

  contact.surface.soft_cfm = 1.0 / (this->stepTimeDouble * kp + kd);

  // contact.surface.soft_erp = 0.5*(_collision1->surface->softERP +
  //                                _collision2->surface->softERP);
  // contact.surface.soft_cfm = 0.5*(_collision1->surface->softCFM +
  //                                _collision2->surface->softCFM);

  // assign fdir1 if not set as 0
  math::Vector3 fd =
    (_collision1->GetSurface()->fdir1 + _collision2->GetSurface()->fdir1) * 0.5;

  if (fd != math::Vector3::Zero)
  {
    contact.surface.mode |= dContactFDir1;
    contact.fdir1[0] = fd.x;
    contact.fdir1[1] = fd.y;
    contact.fdir1[2] = fd.z;
  }

  // Set the friction coefficients.
  contact.surface.mu = std::min(_collision1->GetSurface()->mu1,
                                _collision2->GetSurface()->mu1);
  contact.surface.mu2 = std::min(_collision1->GetSurface()->mu2,
                                 _collision2->GetSurface()->mu2);


  // Set the slip values
  contact.surface.slip1 = std::min(_collision1->GetSurface()->slip1,
                                   _collision2->GetSurface()->slip1);
  contact.surface.slip2 = std::min(_collision1->GetSurface()->slip2,
                                   _collision2->GetSurface()->slip2);

  // Set the bounce values
  contact.surface.bounce = std::min(_collision1->GetSurface()->bounce,
                                    _collision2->GetSurface()->bounce);
  contact.surface.bounce_vel =
    std::min(_collision1->GetSurface()->bounceThreshold,
             _collision2->GetSurface()->bounceThreshold);

  // Get the ODE body IDs
  dBodyID b1 = dGeomGetBody(_collision1->GetCollisionId());
  dBodyID b2 = dGeomGetBody(_collision2->GetCollisionId());

  // Add a new contact to the manager. This will return NULL if no one is
  // listening for contact information.
  Contact *contactFeedback = this->contactManager->NewContact(_collision1,
      _collision2, this->world->GetSimTime());

  ODEJointFeedback *jointFeedback = NULL;

  // Create a joint feedback mechanism
  if (contactFeedback)
  {
    if (this->jointFeedbackIndex < this->jointFeedbacks.size())
      jointFeedback = this->jointFeedbacks[this->jointFeedbackIndex];
    else
    {
      jointFeedback = new ODEJointFeedback();
      this->jointFeedbacks.push_back(jointFeedback);
    }

    this->jointFeedbackIndex++;
    jointFeedback->count = 0;
    jointFeedback->contact = contactFeedback;
  }

  // Create a joint for each contact
  for (int j = 0; j < numc; j++)
  {
    contact.geom = _contactCollisions[this->indices[j]];

    // Create the contact joint. This introduces the contact constraint to
    // ODE
    dJointID contactJoint =
      dJointCreateContact(this->worldId, this->contactGroup, &contact);

    // Store contact information.
    if (contactFeedback && jointFeedback)
    {
      // Store the contact depth
      contactFeedback->depths[j] = _contactCollisions[this->indices[j]].depth;

      // Store the contact position
      contactFeedback->positions[j].Set(
          _contactCollisions[this->indices[j]].pos[0],
          _contactCollisions[this->indices[j]].pos[1],
          _contactCollisions[this->indices[j]].pos[2]);

      // Store the contact normal
      contactFeedback->normals[j].Set(
          _contactCollisions[this->indices[j]].normal[0],
          _contactCollisions[this->indices[j]].normal[1],
          _contactCollisions[this->indices[j]].normal[2]);

      // Set the joint feedback.
      dJointSetFeedback(contactJoint, &(jointFeedback->feedbacks[j]));

      // Increase the counters
      contactFeedback->count++;
      jointFeedback->count++;
    }

    // Attach the contact joint.
    dJointAttach(contactJoint, b1, b2);
  }
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

/////////////////////////////////////////////////
void ODEPhysics::SetSeed(uint32_t _seed)
{
  dRandSetSeed(_seed);
}
