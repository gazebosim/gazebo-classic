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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 */

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "sdf/parser/parser.hh"
#include "transport/Node.hh"
#include "transport/Transport.hh"
#include "transport/Publisher.hh"
#include "transport/Subscriber.hh"

#include "common/Diagnostics.hh"
#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/Plugin.hh"

#include "physics/Link.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/Model.hh"
#include "physics/World.hh"

#include "sensors/Sensor.hh"
#include "sensors/SensorManager.hh"
#include "sensors/CameraSensor.hh"

#include "physics/Collision.hh"

using namespace gazebo;
using namespace physics;


class ModelUpdate_TBB
{
  public: ModelUpdate_TBB(Model_V *models) : models(models) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      (*models)[i]->Update();
    }
  }

  private: Model_V *models;
};

////////////////////////////////////////////////////////////////////////////////
// constructor
World::World(const std::string &_name)
{
  this->receiveMutex = new boost::mutex();

  this->needsReset = false;
  this->stepInc = false;
  this->pause = false;
  this->thread = NULL;

  this->name = _name;

  this->modelWorldPoseUpdateMutex = new boost::recursive_mutex();

  this->connections.push_back(
     event::Events::ConnectStep(boost::bind(&World::OnStep, this)));
  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&World::SetSelectedEntityCB, this, _1)));
  this->connections.push_back(
     event::Events::ConnectDeleteEntity(
       boost::bind(&World::DeleteEntityCB, this, _1)));
}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  delete this->modelWorldPoseUpdateMutex;
  this->modelWorldPoseUpdateMutex = NULL;

  this->connections.clear();
  this->Fini();

  this->sdf->Reset();
  this->rootElement.reset();
  this->node.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load( sdf::ElementPtr _sdf )
{
  this->sdf = _sdf;

  if (this->sdf->GetValueString("name").empty())
    gzwarn << "create_world(world_name=[" 
           << this->name << "]) overwrites sdf world name\n!";
  else
    this->name = this->sdf->GetValueString("name");

  this->sceneMsg.CopyFrom( 
        msgs::SceneFromSDF(_sdf->GetElement("scene")) );

  // The period at which statistics about the world are published
  this->statPeriod = common::Time(0,200000000);


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->GetName());

  this->guiPub = this->node->Advertise<msgs::GUI>("~/gui",1,true);
  if (this->sdf->HasElement("gui"))
    this->guiPub->Publish(msgs::GUIFromSDF(this->sdf->GetElement("gui")));

  this->factorySub = this->node->Subscribe("~/factory", 
                                           &World::OnFactoryMsg, this);
  this->controlSub = this->node->Subscribe("~/world_control", 
                                           &World::OnControl, this);
  this->requestSub = this->node->Subscribe("~/request",&World::OnRequest, this);
  this->jointSub = this->node->Subscribe("~/joint", &World::JointLog, this);
  this->modelSub = this->node->Subscribe<msgs::Model>("~/model/modify",
      &World::OnModelMsg, this);

  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");
  this->responsePub = this->node->Advertise<msgs::Response>("~/response");
  this->statPub =
    this->node->Advertise<msgs::WorldStatistics>("~/world_stats",1);
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection",1);
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/info");

  {
    // Make all incoming messages wait until the world is done loading.
    //boost::mutex::scoped_lock lock(*this->receiveMutex);

    std::string type = _sdf->GetElement("physics")->GetValueString("type");
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine( type, 
        shared_from_this());

    if (this->physicsEngine == NULL)
      gzthrow("Unable to create physics engine\n");

    // This should come before loading of entities
    this->physicsEngine->Load( _sdf->GetElement("physics") );

    this->rootElement.reset(new Base(BasePtr()));
    this->rootElement->SetName(this->GetName());
    WorldPtr me = shared_from_this();
    this->rootElement->SetWorld(me);

    // Create all the entities
    this->LoadEntities(_sdf, this->rootElement);

    // TODO: Performance test to see if TBB model updating is necessary
    // Choose threaded or unthreaded model updating depending on the number of
    // models in the scene
    //if (this->GetModelCount() < 20)
    this->modelUpdateFunc = &World::ModelUpdateSingleLoop;
    //else
    //this->modelUpdateFunc = &World::ModelUpdateTBB;

    event::Events::worldCreated(this->GetName());
  }
}

void World::Save(const std::string &_filename)
{
  this->sdf->Update();
  std::string data;
  data = "<?xml version='1.0'?>\n";
  data += "<gazebo version='1.0'>\n";
  data += this->sdf->ToString("");
  data += "</gazebo>\n";

  std::ofstream out(_filename.c_str(), std::ios::out);
  if (!out)
    gzerr << "Unable to open file[" << _filename << "]\n";
  else
    out << data;

  out.close();
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the world
void World::Init()
{
  // Initialize all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Init();

  // Initialize the physics engine
  this->physicsEngine->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Run the world in a thread
void World::Run()
{
  this->stop = false;
  this->thread = new boost::thread( 
      boost::bind(&World::RunLoop, this));
}
 
////////////////////////////////////////////////////////////////////////////////
// Stop the world
void World::Stop()
{
  this->stop = true;
  if (this->thread)
  {
    this->thread->join();
    delete this->thread;
    this->thread = NULL;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Function to run physics. Used by physicsThread
void World::RunLoop()
{
  this->physicsEngine->InitForThread();

  common::Time step = this->physicsEngine->GetStepTime();

  this->startTime = common::Time::GetWallTime();

  // This fixes a minor issue when the world is paused before it's started
  if (this->IsPaused())
    this->pauseStartTime = this->startTime;

  common::Time prevUpdateTime = common::Time::GetWallTime();
  while (!this->stop)
  {
    /// Send statistics about the world simulation
    if (common::Time::GetWallTime() - this->prevStatTime > this->statPeriod)
    {
      msgs::Set( this->worldStatsMsg.mutable_sim_time(), this->GetSimTime());
      msgs::Set( this->worldStatsMsg.mutable_real_time(), this->GetRealTime() );
      msgs::Set( this->worldStatsMsg.mutable_pause_time(),this->GetPauseTime());
      this->worldStatsMsg.set_paused( this->IsPaused() ); 

      this->statPub->Publish( this->worldStatsMsg );
      this->prevStatTime = common::Time::GetWallTime();
    }

    if (this->IsPaused() && !this->stepInc)
      this->pauseTime += step;
    else
    {
      // throttling update rate
      if (common::Time::GetWallTime() - prevUpdateTime
             >= common::Time(this->physicsEngine->GetUpdatePeriod()))
      {
        prevUpdateTime = common::Time::GetWallTime();
        this->simTime += step;
        this->Update();
      }
    }

    // TODO: Fix timeout:  this belongs in simulator.cc
    /*if (this->timeout > 0 && this->GetRealTime() > this->timeout)
    {
      this->stop = true;
      break;
    }*/

    if (this->IsPaused() && this->stepInc)
      this->stepInc = false;

    this->ProcessEntityMsgs();
    this->ProcessRequestMsgs();
    this->ProcessFactoryMsgs();
    this->ProcessModelMsgs();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the world
void World::Update()
{
  if (this->needsReset)
  {
    this->Reset();
    this->needsReset = false;
  }

  event::Events::worldUpdateStart();

  // Update all the models
  (*this.*modelUpdateFunc)();

  // TODO: put back in
  //Logger::Instance()->Update();

  // Update the physics engine
  if (this->physicsEngine)
  {
    this->physicsEngine->UpdateCollision();

    for (std::list<Entity*>::iterator iter = this->dirtyPoses.begin(); 
        iter != this->dirtyPoses.end(); iter++)
    {
      (*iter)->SetWorldPose( (*iter)->GetDirtyPose(), false );
    }
    this->dirtyPoses.clear();

    this->physicsEngine->UpdatePhysics();
  }

  event::Events::worldUpdateEnd();
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
void World::Fini()
{
  this->Stop();
  this->plugins.clear();

  this->node->Fini();

  if (this->rootElement)
  {
    this->rootElement->Fini();
    this->rootElement.reset();
  }

  if (this->physicsEngine)
  {
    this->physicsEngine->Fini();
    this->physicsEngine.reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Remove all entities from the world
void World::Clear()
{
  // TODO: Implement this
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the world
std::string World::GetName() const
{
  return this->name;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of parameters
unsigned int World::GetParamCount() const
{
  return this->parameters.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param
common::Param *World::GetParam(unsigned int index) const
{
  if (index < this->parameters.size())
    return this->parameters[index];
  else
    gzerr << "World::GetParam - Invalid param index\n";

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Return the physics engine
PhysicsEnginePtr World::GetPhysicsEngine() const
{
  return this->physicsEngine;
}

////////////////////////////////////////////////////////////////////////////////
/// Delete an entity by name
void World::DeleteEntityCB(const std::string &/*_name*/)
{
  // TODO: Implement this function
}

// Get an element by name
BasePtr World::GetByName(const std::string &_name)
{
  return this->rootElement->GetByName(_name);
}

ModelPtr World::GetModelById(unsigned int _id)
{
  return boost::shared_dynamic_cast<Model>(this->rootElement->GetById(_id));
}

/// Get a model by name
ModelPtr World::GetModelByName(const std::string &_name)
{
  return boost::shared_dynamic_cast<Model>(this->GetByName(_name));
}

////////////////////////////////////////////////////////////////////////////////
// Get a pointer to a model based on a name
EntityPtr World::GetEntityByName(const std::string &_name)
{
  return boost::shared_dynamic_cast<Entity>(this->GetByName(_name));
}


////////////////////////////////////////////////////////////////////////////////
// Load a model 
ModelPtr World::LoadModel( sdf::ElementPtr &_sdf , BasePtr _parent)
{
  ModelPtr model( new Model(_parent) );
  model->SetWorld(shared_from_this());
  model->Load(_sdf);

  event::Events::addEntity(model->GetCompleteScopedName());

  msgs::Model msg;
  model->FillModelMsg(msg);
  this->modelPub->Publish(msg);

  return model;
}

///////////////////////////////////////////////////////////////////////////////
// Load a model
void World::LoadEntities( sdf::ElementPtr &_sdf, BasePtr _parent )
{
  if (_sdf->HasElement("light"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("light");
    while (childElem)
    {
      msgs::Light *lm = this->sceneMsg.add_light();
      lm->CopyFrom( msgs::LightFromSDF(childElem) );

      childElem = childElem->GetNextElement();
    }
  }

  if (_sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("model");

    while (childElem)
    {
      this->LoadModel(childElem, _parent);

      // TODO : Put back in the ability to nest models. We should do this
      // without requiring a joint.

      childElem = childElem->GetNextElement();
    }
  }

  // Load the plugins
  if (_sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement();
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Get the number of models
unsigned int World::GetModelCount() const
{
  return this->rootElement->GetChildCount();
}

///////////////////////////////////////////////////////////////////////////////
/// Get a model based on an index
ModelPtr World::GetModel(unsigned int _index)
{
  ModelPtr model;

  if (_index < this->rootElement->GetChildCount() &&
      this->rootElement->GetChild(_index)->HasType(Base::MODEL))
    model = boost::shared_static_cast<Model>(this->rootElement->GetChild(_index));
  else
    gzerr << "Invalid model index\n";

  return model;
}

///////////////////////////////////////////////////////////////////////////////
// Reset the simulation to the initial settings
void World::Reset(bool _resetTime)
{
  if (_resetTime)
  {
    this->simTime = common::Time(0);
    this->pauseTime = common::Time(0);
    this->startTime = common::Time::GetWallTime();
  }

  this->rootElement->Reset();

  for (std::vector<WorldPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); iter++)
  {
    (*iter)->Reset();
  }
  this->physicsEngine->Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Step callback
void World::OnStep()
{
  this->stepInc = true;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the selected entity
void World::SetSelectedEntityCB(const std::string &_name)
{
  msgs::Selection msg;
  BasePtr base = this->GetByName(_name);
  EntityPtr ent = boost::shared_dynamic_cast<Entity>(base);

  // unselect selectedEntity
  if (this->selectedEntity)
  {
    msg.set_id(this->selectedEntity->GetId());
    msg.set_name(this->selectedEntity->GetCompleteScopedName());
    msg.set_selected(false);
    this->selectionPub->Publish(msg);

    this->selectedEntity->SetSelected(false);
  }

  // if a different entity is selected, show bounding box and SetSelected(true)
  if (ent && this->selectedEntity != ent)
  {
    // set selected entity to ent
    this->selectedEntity = ent;
    this->selectedEntity->SetSelected(true);

    msg.set_id(this->selectedEntity->GetId());
    msg.set_name(this->selectedEntity->GetCompleteScopedName());
    msg.set_selected( true );

    this->selectionPub->Publish(msg);
  }
  else
    this->selectedEntity.reset();
  //event::Events::entitySelected(this->selectedEntity);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the selected entity
EntityPtr World::GetSelectedEntity() const
{
  return this->selectedEntity;
}

////////////////////////////////////////////////////////////////////////////////
/// Print entity tree
void World::PrintEntityTree()
{
  // Initialize all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Print("");
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulation time
gazebo::common::Time World::GetSimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the sim time
void World::SetSimTime(common::Time t)
{
  this->simTime = t;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pause time
gazebo::common::Time World::GetPauseTime() const
{
  return this->pauseTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the start time
gazebo::common::Time World::GetStartTime() const
{
  return this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the real time (elapsed time)
common::Time World::GetRealTime() const
{
  if (this->pause)
    return (this->pauseStartTime - this->startTime) - this->realTimeOffset;
  else
    return (common::Time::GetWallTime() - this->startTime) - this->realTimeOffset;
}

////////////////////////////////////////////////////////////////////////////////
// Return when this simulator is paused
bool World::IsPaused() const
{
  return this->pause;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the simulation is paused
void World::SetPaused(bool p)
{
  if (this->pause == p)
    return;

  if (p)
    this->pauseStartTime = common::Time::GetWallTime();
  else
    this->realTimeOffset += common::Time::GetWallTime() - this->pauseStartTime;

  event::Events::pause(p);
  this->pause = p;
}


// Received a factory msg
void World::OnFactoryMsg(ConstFactoryPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->factoryMsgs.push_back(*_msg);
}

void World::OnControl(ConstWorldControlPtr &data)
{
  if (data->has_pause())
    this->SetPaused(data->pause());

  if (data->has_step())
    this->OnStep();

  if (data->has_reset_time())
  {
    this->simTime = common::Time(0);
    this->pauseTime = common::Time(0);
    this->startTime = common::Time::GetWallTime();
  }

  if (data->has_reset_world())
  {
    this->needsReset = true;
  }
}

void World::OnRequest(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->requestMsgs.push_back(*_msg);
}

void World::JointLog(ConstJointPtr &msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  int i = 0;
  for (; i < this->sceneMsg.joint_size(); i++)
  {
    if (this->sceneMsg.joint(i).name() == msg->name())
    {
      this->sceneMsg.mutable_joint(i)->CopyFrom(*msg);
      break;
    }
  }

  if (i >= this->sceneMsg.joint_size())
  {
    msgs::Joint *newJoint = this->sceneMsg.add_joint();
    newJoint->CopyFrom(*msg);
  }
}

void World::OnModelMsg(ConstModelPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->modelMsgs.push_back(*_msg);
}

// Construct a scene message
void World::BuildSceneMsg(msgs::Scene &scene, BasePtr entity)
{
  if (entity)
  {
    if (entity->HasType(Entity::MODEL))
    {
      msgs::Model *modelMsg = scene.add_model();
      boost::shared_static_cast<Model>(entity)->FillModelMsg(*modelMsg);
    }

    for (unsigned int i=0; i < entity->GetChildCount(); i++)
    {
      this->BuildSceneMsg(scene, entity->GetChild(i));
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// TBB version of model updating
/*void World::ModelUpdateTBB()
{
  tbb::parallel_for( tbb::blocked_range<size_t>(0, this->models.size(), 10),
      ModelUpdate_TBB(&this->models) );
}*/

////////////////////////////////////////////////////////////////////////////////
// Single loop verison of model updating
void World::ModelUpdateSingleLoop()
{
  // Update all the models
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Update();
}



////////////////////////////////////////////////////////////////////////////////
// Load a plugin
void World::LoadPlugin( sdf::ElementPtr &_sdf )
{
  std::string name = _sdf->GetValueString("name");
  std::string filename = _sdf->GetValueString("filename");
  gazebo::WorldPluginPtr plugin = gazebo::WorldPlugin::Create(filename, name);
  if (plugin)
  {
    WorldPtr myself = shared_from_this();
    plugin->Load(myself,_sdf);
    this->plugins.push_back( plugin );
  }
}

////////////////////////////////////////////////////////////////////////////////
void World::ProcessEntityMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::list<std::string>::iterator iter;
  for (iter = this->deleteEntity.begin(); 
       iter != this->deleteEntity.end(); iter++)
  {
    this->rootElement->RemoveChild( (*iter) );
  }

  this->deleteEntity.clear();
}

void World::ProcessRequestMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Response response;

  std::list<msgs::Request>::iterator iter;
  for (iter = this->requestMsgs.begin();
       iter != this->requestMsgs.end(); iter++)
  {
    bool send = true;
    response.set_id((*iter).id());
    response.set_request((*iter).request());
    response.set_response("success");

    if ((*iter).request() == "entity_list")
    {
      msgs::Model_V modelVMsg;

      for (unsigned int i=0; i < this->rootElement->GetChildCount(); i++)
      {
        BasePtr entity = this->rootElement->GetChild(i);
        msgs::Model *modelMsg = modelVMsg.add_models();
        if (entity->HasType(Base::MODEL))
        {
          ModelPtr model = boost::shared_dynamic_cast<Model>(entity);
          model->FillModelMsg( *modelMsg );
        }
      }

      response.set_type(modelVMsg.GetTypeName());
      std::string *serializedData = response.mutable_serialized_data();
      modelVMsg.SerializeToString(serializedData);
    }
    else if ((*iter).request() == "entity_delete")
    {
      this->deleteEntity.push_back((*iter).data());
    }
    else if ((*iter).request() == "entity_info")
    {
      BasePtr entity = this->rootElement->GetByName( (*iter).data() );
      if (entity)
      {
        if (entity->HasType(Base::MODEL))
        {
          msgs::Model modelMsg;
          ModelPtr model = boost::shared_dynamic_cast<Model>(entity);
          model->FillModelMsg(modelMsg);

          std::string *serializedData = response.mutable_serialized_data();
          modelMsg.SerializeToString( serializedData );
          response.set_type( modelMsg.GetTypeName() );
        }
        else if (entity->HasType(Base::LINK))
        {
          msgs::Link linkMsg;
          LinkPtr link = boost::shared_dynamic_cast<Link>(entity);
          link->FillLinkMsg(linkMsg);

          std::string *serializedData = response.mutable_serialized_data();
          linkMsg.SerializeToString( serializedData );
          response.set_type( linkMsg.GetTypeName() );
        }
        else if (entity->HasType(Base::COLLISION))
        {
          msgs::Collision collisionMsg;
          CollisionPtr collision = boost::shared_dynamic_cast<Collision>(entity);
          collision->FillCollisionMsg( collisionMsg );

          std::string *serializedData = response.mutable_serialized_data();
          collisionMsg.SerializeToString( serializedData );
          response.set_type( collisionMsg.GetTypeName() );
        }
        else if (entity->HasType(Base::JOINT))
        {
          msgs::Joint jointMsg;
          JointPtr joint = boost::shared_dynamic_cast<Joint>(entity);
          joint->FillJointMsg( jointMsg );

          std::string *serializedData = response.mutable_serialized_data();
          jointMsg.SerializeToString( serializedData );
          response.set_type( jointMsg.GetTypeName() );
        }
      }
      else
      {
        response.set_type("error");
        response.set_response( "nonexistant" );
      }
    }
    else if ((*iter).request() == "scene_info")
    {
      this->sceneMsg.clear_model();
      this->BuildSceneMsg(this->sceneMsg, this->rootElement);

      std::string *serializedData = response.mutable_serialized_data();
      this->sceneMsg.SerializeToString(serializedData);
      response.set_type( sceneMsg.GetTypeName() );
    }
    else
      send = false;

    if (send)
      this->responsePub->Publish(response);
  }

  this->requestMsgs.clear();
}

void World::ProcessModelMsgs()
{
  std::list<msgs::Model>::iterator iter;
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  for (iter = this->modelMsgs.begin();
       iter != this->modelMsgs.end(); iter++)
  {
    ModelPtr model = this->GetModelById((*iter).id());
    if (!model)
      gzerr << "Unable to find model[" << (*iter).name() << "] Id[" << (*iter).id() << "]\n";
    else
    {
      model->ProcessMsg(*iter);

      // Let all other subscribers know about the change
      msgs::Model msg;
      model->FillModelMsg(msg);
      this->modelPub->Publish(msg);
    }
  }
  this->modelMsgs.clear();
}

void World::ProcessFactoryMsgs()
{
  std::list<msgs::Factory>::iterator iter;
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  for (iter = this->factoryMsgs.begin();
       iter != this->factoryMsgs.end(); iter++)
  {
    sdf::SDFPtr factorySDF(new sdf::SDF);
    sdf::initFile( "/sdf/gazebo.sdf", factorySDF );

    if ((*iter).has_sdf() && !(*iter).sdf().empty())
    {
      // SDF Parsing happens here
      if (!sdf::readString( (*iter).sdf(), factorySDF))
      {
        gzerr << "Unable to read sdf string\n";
        continue;
      }

    }
    else if ((*iter).has_sdf_filename() && !(*iter).sdf_filename().empty())
    {
      if (!sdf::readFile( (*iter).sdf_filename(), factorySDF))
      {
        gzerr << "Unable to read sdf file.\n";
        continue;
      }
    }
    else
    {
      gzerr << "Unable to load sdf from factory message."
        << "No SDF or SDF filename specified.\n";
    }

    if ((*iter).has_edit_name())
    {
      factorySDF->PrintValues();

      BasePtr base = this->rootElement->GetByName( (*iter).edit_name() );
      if (base)
      {
        sdf::ElementPtr elem;
        if (factorySDF->root->GetName() == "gazebo")
          elem = factorySDF->root->GetFirstElement();
        else
          elem = factorySDF->root;

        base->UpdateParameters( elem );
      }
    }
    else
    {
      sdf::ElementPtr elem = factorySDF->root->GetElement("model");
      if (!elem) 
        elem = factorySDF->root->GetElement("world")->GetElement("model");

      elem->SetParent(this->sdf);
      elem->GetParent()->InsertElement(elem);
      ModelPtr model = this->LoadModel( elem, this->rootElement );
      if ((*iter).has_pose())
        model->SetWorldPose( msgs::Convert( (*iter).pose() ) );

      model->Init();
    }

    factorySDF->root.reset();
  }

  this->factoryMsgs.clear();
}
