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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 */

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/sdf/sdf.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/Common.hh"
#include "gazebo/common/Diagnostics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Plugin.hh"

#include "gazebo/physics/Road.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/World.hh"

#include "physics/Collision.hh"

using namespace gazebo;
using namespace physics;


class ModelUpdate_TBB
{
  public: ModelUpdate_TBB(Model_V *_models) : models(_models) {}
  public: void operator() (const tbb::blocked_range<size_t> &_r) const
  {
    for (size_t i = _r.begin(); i != _r.end(); i++)
    {
      (*models)[i]->Update();
    }
  }

  private: Model_V *models;
};

//////////////////////////////////////////////////
World::World(const std::string &_name)
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("world.sdf", this->sdf);

  this->receiveMutex = new boost::mutex();
  this->loadModelMutex = new boost::mutex();

  this->initialized = false;
  this->stepInc = 0;
  this->pause = false;
  this->thread = NULL;

  this->pluginsLoaded = false;

  this->name = _name;

  this->needsReset = false;
  this->resetAll = true;
  this->resetTimeOnly = false;
  this->resetModelOnly = false;
  this->enablePhysicsEngine = true;

  this->setWorldPoseMutex = new boost::mutex();
  this->worldUpdateMutex = new boost::recursive_mutex();

  this->connections.push_back(
     event::Events::ConnectStep(boost::bind(&World::OnStep, this)));
  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&World::SetSelectedEntityCB, this, _1)));
}

//////////////////////////////////////////////////
World::~World()
{
  delete this->receiveMutex;
  this->receiveMutex = NULL;
  delete this->loadModelMutex;
  this->loadModelMutex = NULL;
  delete this->setWorldPoseMutex;
  this->setWorldPoseMutex = NULL;
  delete this->worldUpdateMutex;
  this->worldUpdateMutex = NULL;

  this->connections.clear();
  this->Fini();

  this->sdf->Reset();
  this->rootElement.reset();
  this->node.reset();

  this->testRay.reset();
}

//////////////////////////////////////////////////
void World::Load(sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;

  if (this->sdf->GetValueString("name").empty())
    gzwarn << "create_world(world_name =["
           << this->name << "]) overwrites sdf world name\n!";
  else
    this->name = this->sdf->GetValueString("name");

  this->sceneMsg.CopyFrom(msgs::SceneFromSDF(this->sdf->GetElement("scene")));
  this->sceneMsg.set_name(this->GetName());

  // The period at which statistics about the world are published
  this->statPeriod = common::Time(0, 200000000);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->GetName());

  this->guiPub = this->node->Advertise<msgs::GUI>("~/gui", 1, true);
  if (this->sdf->HasElement("gui"))
    this->guiPub->Publish(msgs::GUIFromSDF(this->sdf->GetElement("gui")));

  this->factorySub = this->node->Subscribe("~/factory",
                                           &World::OnFactoryMsg, this);
  this->controlSub = this->node->Subscribe("~/world_control",
                                           &World::OnControl, this);
  this->requestSub = this->node->Subscribe("~/request",
                                           &World::OnRequest, this);
  this->jointSub = this->node->Subscribe("~/joint", &World::JointLog, this);
  this->modelSub = this->node->Subscribe<msgs::Model>("~/model/modify",
      &World::OnModelMsg, this);

  this->responsePub = this->node->Advertise<msgs::Response>("~/response");
  this->statPub =
    this->node->Advertise<msgs::WorldStatistics>("~/world_stats", 1);
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection", 1);
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/info");
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  std::string type = this->sdf->GetElement("physics")->GetValueString("type");
  this->physicsEngine = PhysicsFactory::NewPhysicsEngine(type,
      shared_from_this());

  if (this->physicsEngine == NULL)
    gzthrow("Unable to create physics engine\n");

  // This should come before loading of entities
  this->physicsEngine->Load(this->sdf->GetElement("physics"));

  this->rootElement.reset(new Base(BasePtr()));
  this->rootElement->SetName(this->GetName());
  this->rootElement->SetWorld(shared_from_this());

  if (this->sdf->HasElement("state"))
  {
    sdf::ElementPtr childElem = this->sdf->GetElement("state");

    while (childElem)
    {
      WorldState state;
      state.Load(childElem);
      this->sdf->InsertElement(childElem);
      this->UpdateSDFFromState(state);
      // this->SetState(state);

      childElem = childElem->GetNextElement("state");

      // TODO: We currently load just the first state data. Need to
      // implement a better mechanism for handling multiple states
      break;
    }
  }

  // Create all the entities
  this->LoadEntities(this->sdf, this->rootElement);

  // TODO: Performance test to see if TBB model updating is necessary
  // Choose threaded or unthreaded model updating depending on the number of
  // models in the scene
  // if (this->GetModelCount() < 20)
  this->modelUpdateFunc = &World::ModelUpdateSingleLoop;
  // else
  // this->modelUpdateFunc = &World::ModelUpdateTBB;

  event::Events::worldCreated(this->GetName());
}


//////////////////////////////////////////////////
void World::Save(const std::string &_filename)
{
  this->UpdateStateSDF();
  std::string data;
  data = "<?xml version ='1.0'?>\n";
  data += "<gazebo version ='";
  data += SDF_VERSION;
  data += "'>\n";
  data += this->sdf->ToString("");
  data += "</gazebo>\n";

  std::ofstream out(_filename.c_str(), std::ios::out);
  if (!out)
    gzerr << "Unable to open file[" << _filename << "]\n";
  else
    out << data;

  out.close();
}

//////////////////////////////////////////////////
void World::Init()
{
  // Initialize all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Init();

  // Initialize the physics engine
  this->physicsEngine->Init();

  this->testRay = boost::shared_dynamic_cast<RayShape>(
      this->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  this->initialized = true;
}

//////////////////////////////////////////////////
void World::Run()
{
  this->stop = false;
  this->thread = new boost::thread(
      boost::bind(&World::RunLoop, this));
}

//////////////////////////////////////////////////
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


//////////////////////////////////////////////////
void World::RunLoop()
{
  this->physicsEngine->InitForThread();

  this->startTime = common::Time::GetWallTime();

  // This fixes a minor issue when the world is paused before it's started
  if (this->IsPaused())
    this->pauseStartTime = this->startTime;

  this->prevStepWallTime = common::Time::GetWallTime();

  while (!this->stop)
    this->Step();
}

//////////////////////////////////////////////////
void World::Step()
{
  this->worldUpdateMutex->lock();

  // Send statistics about the world simulation
  if (common::Time::GetWallTime() - this->prevStatTime > this->statPeriod)
  {
    msgs::Set(this->worldStatsMsg.mutable_sim_time(), this->GetSimTime());
    msgs::Set(this->worldStatsMsg.mutable_real_time(), this->GetRealTime());
    msgs::Set(this->worldStatsMsg.mutable_pause_time(), this->GetPauseTime());
    this->worldStatsMsg.set_paused(this->IsPaused());

    this->statPub->Publish(this->worldStatsMsg);
    this->prevStatTime = common::Time::GetWallTime();
  }

  if (this->IsPaused() && !this->stepInc > 0)
    this->pauseTime += this->physicsEngine->GetStepTime();
  else
  {
    // throttling update rate
    if (common::Time::GetWallTime() - this->prevStepWallTime
           >= common::Time(this->physicsEngine->GetUpdatePeriod()))
    {
      this->prevStepWallTime = common::Time::GetWallTime();
      // query timestep to allow dynamic time step size updates
      this->simTime += this->physicsEngine->GetStepTime();
      this->Update();
    }
  }

  // TODO: Fix timeout:  this belongs in simulator.cc
  /*if (this->timeout > 0 && this->GetRealTime() > this->timeout)
  {
    this->stop = true;
    break;
  }*/

  if (this->IsPaused() && this->stepInc > 0)
    this->stepInc--;

  this->ProcessEntityMsgs();
  this->ProcessRequestMsgs();
  this->ProcessFactoryMsgs();
  this->ProcessModelMsgs();
  this->worldUpdateMutex->unlock();
}

//////////////////////////////////////////////////
void World::StepWorld(int _steps)
{
  if (!this->IsPaused())
  {
    gzwarn << "Calling World::StepWorld(steps) while world is not paused\n";
    this->SetPaused(true);
  }

  this->worldUpdateMutex->lock();
  this->stepInc = _steps;
  this->worldUpdateMutex->unlock();

  // block on completion
  bool wait = true;
  while (wait)
  {
    common::Time::MSleep(1);
    this->worldUpdateMutex->lock();
    if (this->stepInc == 0 || this->stop)
      wait = false;
    this->worldUpdateMutex->unlock();
  }
}

//////////////////////////////////////////////////
void World::Update()
{
  if (this->needsReset)
  {
    if (this->resetAll)
      this->Reset();
    else if (this->resetTimeOnly)
      this->ResetTime();
    else if (this->resetModelOnly)
      this->ResetEntities(Base::MODEL);
    this->needsReset = false;
  }

  event::Events::worldUpdateStart();

  // Update all the models
  (*this.*modelUpdateFunc)();

  // TODO: put back in
  // Logger::Instance()->Update();

  // Update the physics engine
  if (this->enablePhysicsEngine && this->physicsEngine)
  {
    this->physicsEngine->UpdateCollision();

    this->physicsEngine->UpdatePhysics();

    /// need this because ODE does not call dxReallocateWorldProcessContext()
    /// until dWorld.*Step
    /// Plugins that manipulate joints (and probably other properties) require
    /// one iteration of the physics engine. Do not remove this.
    if (!this->pluginsLoaded)
    {
      this->LoadPlugins();
      this->pluginsLoaded = true;
    }

    // do this after physics update as
    //   ode --> MoveCallback sets the dirtyPoses
    //           and we need to propagate it into Entity::worldPose
    for (std::list<Entity*>::iterator iter = this->dirtyPoses.begin();
        iter != this->dirtyPoses.end(); ++iter)
    {
      (*iter)->SetWorldPose((*iter)->GetDirtyPose(), false);
    }

    this->dirtyPoses.clear();
  }

  event::Events::worldUpdateEnd();
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
void World::Clear()
{
  // TODO: Implement this
}

//////////////////////////////////////////////////
std::string World::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
PhysicsEnginePtr World::GetPhysicsEngine() const
{
  return this->physicsEngine;
}

//////////////////////////////////////////////////
BasePtr World::GetByName(const std::string &_name)
{
  return this->rootElement->GetByName(_name);
}

/////////////////////////////////////////////////
ModelPtr World::GetModelById(unsigned int _id)
{
  return boost::shared_dynamic_cast<Model>(this->rootElement->GetById(_id));
}

//////////////////////////////////////////////////
ModelPtr World::GetModelByName(const std::string &_name)
{
  return boost::shared_dynamic_cast<Model>(this->GetByName(_name));
}

//////////////////////////////////////////////////
ModelPtr World::GetModel(const std::string &_name)
{
  boost::mutex::scoped_lock lock(*this->loadModelMutex);
  return boost::shared_dynamic_cast<Model>(this->GetByName(_name));
}

//////////////////////////////////////////////////
EntityPtr World::GetEntityByName(const std::string &_name)
{
  return boost::shared_dynamic_cast<Entity>(this->GetByName(_name));
}

//////////////////////////////////////////////////
EntityPtr World::GetEntity(const std::string &_name)
{
  return boost::shared_dynamic_cast<Entity>(this->GetByName(_name));
}

//////////////////////////////////////////////////
ModelPtr World::LoadModel(sdf::ElementPtr _sdf , BasePtr _parent)
{
  boost::mutex::scoped_lock lock(*this->loadModelMutex);
  ModelPtr model;

  if (_sdf->GetName() == "model")
  {
    model.reset(new Model(_parent));
    model->SetWorld(shared_from_this());
    model->Load(_sdf);

    event::Events::addEntity(model->GetScopedName());

    msgs::Model msg;
    model->FillModelMsg(msg);
    this->modelPub->Publish(msg);
  }
  else
  {
    gzerr << "SDF is missing the <model> tag:\n";
    _sdf->PrintValues("  ");
  }

  return model;
}

//////////////////////////////////////////////////
ActorPtr World::LoadActor(sdf::ElementPtr _sdf , BasePtr _parent)
{
  ActorPtr actor(new Actor(_parent));
  actor->SetWorld(shared_from_this());
  actor->Load(_sdf);

  event::Events::addEntity(actor->GetScopedName());

  msgs::Model msg;
  actor->FillModelMsg(msg);
  this->modelPub->Publish(msg);

  return actor;
}

//////////////////////////////////////////////////
RoadPtr World::LoadRoad(sdf::ElementPtr _sdf , BasePtr _parent)
{
  RoadPtr road(new Road(_parent));
  road->Load(_sdf);
  return road;
}

//////////////////////////////////////////////////
void World::LoadEntities(sdf::ElementPtr _sdf, BasePtr _parent)
{
  if (_sdf->HasElement("light"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("light");
    while (childElem)
    {
      msgs::Light *lm = this->sceneMsg.add_light();
      lm->CopyFrom(msgs::LightFromSDF(childElem));

      childElem = childElem->GetNextElement("light");
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

      childElem = childElem->GetNextElement("model");
    }
  }

  if (_sdf->HasElement("actor"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("actor");

    while (childElem)
    {
      this->LoadActor(childElem, _parent);

      childElem = childElem->GetNextElement("actor");
    }
  }

  if (_sdf->HasElement("road"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("road");
    while (childElem)
    {
      this->LoadRoad(childElem, _parent);
      childElem = childElem->GetNextElement("road");
    }
  }
}

//////////////////////////////////////////////////
unsigned int World::GetModelCount() const
{
  return this->rootElement->GetChildCount();
}

//////////////////////////////////////////////////
ModelPtr World::GetModel(unsigned int _index) const
{
  ModelPtr model;

  if (_index < this->rootElement->GetChildCount() &&
      this->rootElement->GetChild(_index)->HasType(Base::MODEL))
  {
    model =
      boost::shared_static_cast<Model>(this->rootElement->GetChild(_index));
  }
  else
  {
    gzerr << "Invalid model index\n";
  }

  return model;
}

//////////////////////////////////////////////////
std::list<ModelPtr> World::GetModels() const
{
  std::list<ModelPtr> models;
  for (unsigned int i = 0; i < this->GetModelCount(); ++i)
  {
    models.push_back(this->GetModel(i));
  }

  return models;
}

//////////////////////////////////////////////////
void World::ResetTime()
{
  this->simTime = common::Time(0);
  this->pauseTime = common::Time(0);
  this->startTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
void World::ResetEntities(Base::EntityType _type)
{
  this->rootElement->Reset(_type);
}

//////////////////////////////////////////////////
void World::Reset()
{
  bool currently_paused = this->IsPaused();
  this->SetPaused(true);
  this->worldUpdateMutex->lock();

  this->ResetTime();
  this->ResetEntities(Base::BASE);
  for (std::vector<WorldPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); ++iter)
    (*iter)->Reset();
  this->physicsEngine->Reset();

  this->worldUpdateMutex->unlock();
  this->SetPaused(currently_paused);
}

//////////////////////////////////////////////////
void World::OnStep()
{
  this->stepInc = 1;
}

//////////////////////////////////////////////////
void World::SetSelectedEntityCB(const std::string &_name)
{
  msgs::Selection msg;
  BasePtr base = this->GetByName(_name);
  EntityPtr ent = boost::shared_dynamic_cast<Entity>(base);

  // unselect selectedEntity
  if (this->selectedEntity)
  {
    msg.set_id(this->selectedEntity->GetId());
    msg.set_name(this->selectedEntity->GetScopedName());
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
    msg.set_name(this->selectedEntity->GetScopedName());
    msg.set_selected(true);

    this->selectionPub->Publish(msg);
  }
  else
    this->selectedEntity.reset();
  // event::Events::entitySelected(this->selectedEntity);
}

//////////////////////////////////////////////////
EntityPtr World::GetSelectedEntity() const
{
  return this->selectedEntity;
}

//////////////////////////////////////////////////
void World::PrintEntityTree()
{
  // Initialize all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Print("");
}

//////////////////////////////////////////////////
gazebo::common::Time World::GetSimTime() const
{
  return this->simTime;
}

//////////////////////////////////////////////////
void World::SetSimTime(common::Time _t)
{
  this->simTime = _t;
}

//////////////////////////////////////////////////
gazebo::common::Time World::GetPauseTime() const
{
  return this->pauseTime;
}

//////////////////////////////////////////////////
gazebo::common::Time World::GetStartTime() const
{
  return this->startTime;
}

//////////////////////////////////////////////////
common::Time World::GetRealTime() const
{
  if (this->pause)
    return (this->pauseStartTime - this->startTime) - this->realTimeOffset;
  else
    return (common::Time::GetWallTime() - this->startTime) -
             this->realTimeOffset;
}

//////////////////////////////////////////////////
bool World::IsPaused() const
{
  return this->pause;
}

//////////////////////////////////////////////////
void World::SetPaused(bool _p)
{
  if (this->pause == _p)
    return;

  this->worldUpdateMutex->lock();
  this->pause = _p;
  this->worldUpdateMutex->unlock();

  if (_p)
    this->pauseStartTime = common::Time::GetWallTime();
  else
    this->realTimeOffset += common::Time::GetWallTime() - this->pauseStartTime;

  event::Events::pause(_p);
}

//////////////////////////////////////////////////
void World::OnFactoryMsg(ConstFactoryPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->factoryMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::OnControl(ConstWorldControlPtr &_data)
{
  if (_data->has_pause())
    this->SetPaused(_data->pause());

  if (_data->has_step())
    this->OnStep();

  if (_data->has_reset())
  {
    this->needsReset = true;

    if (_data->reset().has_all() && _data->reset().all())
    {
        this->resetAll = true;
    }
    else
    {
      this->resetAll = false;

      if (_data->reset().has_time_only() && _data->reset().time_only())
        this->resetTimeOnly = true;

      if (_data->reset().has_model_only() && _data->reset().model_only())
        this->resetModelOnly = true;
    }
  }
}

//////////////////////////////////////////////////
void World::OnRequest(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->requestMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::JointLog(ConstJointPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  int i = 0;
  for (; i < this->sceneMsg.joint_size(); i++)
  {
    if (this->sceneMsg.joint(i).name() == _msg->name())
    {
      this->sceneMsg.mutable_joint(i)->CopyFrom(*_msg);
      break;
    }
  }

  if (i >= this->sceneMsg.joint_size())
  {
    msgs::Joint *newJoint = this->sceneMsg.add_joint();
    newJoint->CopyFrom(*_msg);
  }
}

//////////////////////////////////////////////////
void World::OnModelMsg(ConstModelPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->modelMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::BuildSceneMsg(msgs::Scene &_scene, BasePtr _entity)
{
  if (_entity)
  {
    if (_entity->HasType(Entity::MODEL))
    {
      msgs::Model *modelMsg = _scene.add_model();
      boost::shared_static_cast<Model>(_entity)->FillModelMsg(*modelMsg);
    }

    for (unsigned int i = 0; i < _entity->GetChildCount(); ++i)
    {
      this->BuildSceneMsg(_scene, _entity->GetChild(i));
    }
  }
}


//////////////////////////////////////////////////
/*void World::ModelUpdateTBB()
{
  tbb::parallel_for(tbb::blocked_range<size_t>(0, this->models.size(), 10),
      ModelUpdate_TBB(&this->models));
}*/

//////////////////////////////////////////////////
void World::ModelUpdateSingleLoop()
{
  // Update all the models
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Update();
}


//////////////////////////////////////////////////
void World::LoadPlugins()
{
  // Load the plugins
  if (this->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }


  for (std::vector<WorldPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); ++iter)
  {
    (*iter)->Init();
  }
}

//////////////////////////////////////////////////
void World::LoadPlugin(const std::string &_filename,
                       const std::string &_name,
                       sdf::ElementPtr _sdf)
{
  gazebo::WorldPluginPtr plugin = gazebo::WorldPlugin::Create(_filename,
                                                              _name);

  if (plugin)
  {
    if (plugin->GetType() != WORLD_PLUGIN)
    {
      gzerr << "World[" << this->GetName() << "] is attempting to load "
            << "a plugin, but detected an incorrect plugin type. "
            << "Plugin filename[" << _filename << "] name[" << _name << "]\n";
      return;
    }
    plugin->Load(shared_from_this(), _sdf);
    this->plugins.push_back(plugin);

    if (this->initialized)
      plugin->Init();
  }
}

//////////////////////////////////////////////////
void World::RemovePlugin(const std::string &_name)
{
  std::vector<WorldPluginPtr>::iterator iter;
  for (iter = this->plugins.begin(); iter != this->plugins.end(); ++iter)
  {
    if ((*iter)->GetHandle() == _name)
    {
      this->plugins.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void World::LoadPlugin(sdf::ElementPtr _sdf)
{
  std::string pluginName = _sdf->GetValueString("name");
  std::string filename = _sdf->GetValueString("filename");
  this->LoadPlugin(filename, pluginName, _sdf);
}

//////////////////////////////////////////////////
void World::ProcessEntityMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::list<std::string>::iterator iter;
  for (iter = this->deleteEntity.begin();
       iter != this->deleteEntity.end(); ++iter)
  {
    // Remove all the dirty poses from the delete entity.
    for (std::list<Entity*>::iterator iter2 = this->dirtyPoses.begin();
         iter2 != this->dirtyPoses.end();)
    {
      if ((*iter2)->GetName() == *iter ||
          (*iter2)->GetParent()->GetName() == *iter)
      {
        this->dirtyPoses.erase(iter2++);
      }
      else
        ++iter2;
    }

    this->rootElement->RemoveChild((*iter));
  }

  if (this->deleteEntity.size() > 0)
  {
    this->EnableAllModels();
    this->deleteEntity.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessRequestMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Response response;

  std::list<msgs::Request>::iterator iter;
  for (iter = this->requestMsgs.begin();
       iter != this->requestMsgs.end(); ++iter)
  {
    bool send = true;
    response.set_id((*iter).id());
    response.set_request((*iter).request());
    response.set_response("success");

    if ((*iter).request() == "entity_list")
    {
      msgs::Model_V modelVMsg;

      for (unsigned int i = 0; i < this->rootElement->GetChildCount(); ++i)
      {
        BasePtr entity = this->rootElement->GetChild(i);
        if (entity->HasType(Base::MODEL))
        {
          msgs::Model *modelMsg = modelVMsg.add_models();
          ModelPtr model = boost::shared_dynamic_cast<Model>(entity);
          model->FillModelMsg(*modelMsg);
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
      BasePtr entity = this->rootElement->GetByName((*iter).data());
      if (entity)
      {
        if (entity->HasType(Base::MODEL))
        {
          msgs::Model modelMsg;
          ModelPtr model = boost::shared_dynamic_cast<Model>(entity);
          model->FillModelMsg(modelMsg);

          std::string *serializedData = response.mutable_serialized_data();
          modelMsg.SerializeToString(serializedData);
          response.set_type(modelMsg.GetTypeName());
        }
        else if (entity->HasType(Base::LINK))
        {
          msgs::Link linkMsg;
          LinkPtr link = boost::shared_dynamic_cast<Link>(entity);
          link->FillLinkMsg(linkMsg);

          std::string *serializedData = response.mutable_serialized_data();
          linkMsg.SerializeToString(serializedData);
          response.set_type(linkMsg.GetTypeName());
        }
        else if (entity->HasType(Base::COLLISION))
        {
          msgs::Collision collisionMsg;
          CollisionPtr collision =
            boost::shared_dynamic_cast<Collision>(entity);
          collision->FillCollisionMsg(collisionMsg);

          std::string *serializedData = response.mutable_serialized_data();
          collisionMsg.SerializeToString(serializedData);
          response.set_type(collisionMsg.GetTypeName());
        }
        else if (entity->HasType(Base::JOINT))
        {
          msgs::Joint jointMsg;
          JointPtr joint = boost::shared_dynamic_cast<Joint>(entity);
          joint->FillJointMsg(jointMsg);

          std::string *serializedData = response.mutable_serialized_data();
          jointMsg.SerializeToString(serializedData);
          response.set_type(jointMsg.GetTypeName());
        }
      }
      else
      {
        response.set_type("error");
        response.set_response("nonexistant");
      }
    }
    else if ((*iter).request() == "world_sdf")
    {
      msgs::GzString msg;
      this->UpdateStateSDF();
      std::string data;
      data = "<?xml version ='1.0'?>\n";
      data += "<gazebo version ='1.0'>\n";
      data += this->sdf->ToString("");
      data += "</gazebo>\n";
      msg.set_data(data);

      std::string *serializedData = response.mutable_serialized_data();
      msg.SerializeToString(serializedData);
      response.set_type(msg.GetTypeName());
    }
    else if ((*iter).request() == "scene_info")
    {
      this->sceneMsg.clear_model();
      this->BuildSceneMsg(this->sceneMsg, this->rootElement);

      std::string *serializedData = response.mutable_serialized_data();
      this->sceneMsg.SerializeToString(serializedData);
      response.set_type(sceneMsg.GetTypeName());
    }
    else
      send = false;

    if (send)
    {
      this->responsePub->Publish(response);
    }
  }

  this->requestMsgs.clear();
}

//////////////////////////////////////////////////
void World::ProcessModelMsgs()
{
  std::list<msgs::Model>::iterator iter;
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  for (iter = this->modelMsgs.begin();
       iter != this->modelMsgs.end(); ++iter)
  {
    ModelPtr model;
    if ((*iter).has_id())
      model = this->GetModelById((*iter).id());
    else
      model = this->GetModel((*iter).name());

    if (!model)
      gzerr << "Unable to find model["
            << (*iter).name() << "] Id[" << (*iter).id() << "]\n";
    else
    {
      model->ProcessMsg(*iter);

      // Let all other subscribers know about the change
      msgs::Model msg;
      model->FillModelMsg(msg);
      this->modelPub->Publish(msg);
    }
  }
  if (this->modelMsgs.size())
  {
    this->EnableAllModels();
    this->modelMsgs.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessFactoryMsgs()
{
  std::list<msgs::Factory>::iterator iter;
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  for (iter = this->factoryMsgs.begin();
       iter != this->factoryMsgs.end(); ++iter)
  {
    sdf::SDFPtr factorySDF(new sdf::SDF);
    sdf::initFile("gazebo.sdf", factorySDF);

    if ((*iter).has_sdf() && !(*iter).sdf().empty())
    {
      // SDF Parsing happens here
      if (!sdf::readString((*iter).sdf(), factorySDF))
      {
        gzerr << "Unable to read sdf string\n";
        continue;
      }
    }
    else if ((*iter).has_sdf_filename() && !(*iter).sdf_filename().empty())
    {
      std::string filename = common::ModelDatabase::GetModelFile(
          (*iter).sdf_filename());

      if (!sdf::readFile(filename, factorySDF))
      {
        gzerr << "Unable to read sdf file.\n";
        continue;
      }
    }
    else if ((*iter).has_clone_model_name())
    {
      ModelPtr model = this->GetModel((*iter).clone_model_name());
      if (!model)
      {
        gzerr << "Unable to clone model[" << (*iter).clone_model_name()
              << "]. Model not found.\n";
        continue;
      }

      factorySDF->root->InsertElement(model->GetSDF()->Clone());

      std::string newName = model->GetName() + "_clone";
      int i = 0;
      while (this->GetModel(newName))
      {
        newName = model->GetName() + "_clone_" +
                  boost::lexical_cast<std::string>(i);
        i++;
      }

      factorySDF->root->GetElement("model")->GetAttribute("name")->Set(newName);
    }
    else
    {
      gzerr << "Unable to load sdf from factory message."
            << "No SDF or SDF filename specified.\n";
      continue;
    }

    if ((*iter).has_edit_name())
    {
      BasePtr base = this->rootElement->GetByName((*iter).edit_name());
      if (base)
      {
        sdf::ElementPtr elem;
        if (factorySDF->root->GetName() == "gazebo")
          elem = factorySDF->root->GetFirstElement();
        else
          elem = factorySDF->root;

        base->UpdateParameters(elem);
      }
    }
    else
    {
      bool isActor = false;
      bool isModel = false;
      bool isLight = false;

      sdf::ElementPtr elem = factorySDF->root;

      if (elem->HasElement("world"))
        elem = elem->GetElement("world");

      if (elem->HasElement("model"))
      {
        elem = elem->GetElement("model");
        isModel = true;
      }
      else if (elem->HasElement("light"))
      {
        elem = elem->GetElement("light");
        isLight = true;
      }
      else if (elem->HasElement("actor"))
      {
        elem = elem->GetElement("actor");
        isActor = true;
      }
      else
      {
        gzerr << "Unable to find a model, light, or actor in:\n";
        factorySDF->root->PrintValues("");
        continue;
      }

      if (!elem)
      {
        gzerr << "Invalid SDF:";
        factorySDF->root->PrintValues("");
        continue;
      }

      elem->SetParent(this->sdf);
      elem->GetParent()->InsertElement(elem);
      if ((*iter).has_pose())
        elem->GetElement("pose")->Set(msgs::Convert((*iter).pose()));

      if (isActor)
      {
        ActorPtr actor = this->LoadActor(elem, this->rootElement);
        actor->Init();
      }
      else if (isModel)
      {
        ModelPtr model = this->LoadModel(elem, this->rootElement);
        model->Init();
      }
      else if (isLight)
      {
        /// \TODO: Current broken. See Issue #67.
        msgs::Light *lm = this->sceneMsg.add_light();
        lm->CopyFrom(msgs::LightFromSDF(elem));

        this->lightPub->Publish(*lm);
      }
    }
  }

  this->factoryMsgs.clear();
}

//////////////////////////////////////////////////
ModelPtr World::GetModelBelowPoint(const math::Vector3 &_pt)
{
  ModelPtr model;
  EntityPtr entity = this->GetEntityBelowPoint(_pt);

  if (entity)
    model = entity->GetParentModel();
  else
    gzerr << "Unable to find entity below point[" << _pt << "]\n";

  return model;
}

//////////////////////////////////////////////////
EntityPtr World::GetEntityBelowPoint(const math::Vector3 &_pt)
{
  std::string entityName;
  double dist;
  math::Vector3 end;

  end = _pt;
  end.z -= 1000;

  this->physicsEngine->InitForThread();
  this->testRay->SetPoints(_pt, end);
  this->testRay->GetIntersection(dist, entityName);
  return this->GetEntity(entityName);
}

//////////////////////////////////////////////////
WorldState World::GetState()
{
  return WorldState(shared_from_this());
}

//////////////////////////////////////////////////
void World::UpdateStateSDF()
{
  this->sdf->Update();
  sdf::ElementPtr stateElem = this->sdf->GetElement("state");
  stateElem->ClearElements();

  stateElem->GetAttribute("world_name")->Set(this->GetName());
  stateElem->GetElement("time")->Set(this->GetSimTime());

  for (unsigned int i = 0; i < this->GetModelCount(); ++i)
  {
    sdf::ElementPtr elem = stateElem->AddElement("model");
    this->GetModel(i)->GetState().FillStateSDF(elem);
  }
}

//////////////////////////////////////////////////
void World::SetState(const WorldState &_state)
{
  sdf::ElementPtr stateElem = this->sdf->GetElement("state");

  stateElem->GetAttribute("world_name")->Set(_state.GetName());
  stateElem->GetElement("time")->Set(_state.GetSimTime());

  this->SetSimTime(_state.GetSimTime());
  for (unsigned int i = 0; i < _state.GetModelStateCount(); ++i)
  {
    ModelState modelState = _state.GetModelState(i);
    ModelPtr model = this->GetModel(modelState.GetName());
    modelState.FillStateSDF(stateElem->AddElement("model"));
    if (model)
      model->SetState(modelState);
    else
      gzerr << "Unable to find model[" << modelState.GetName() << "]\n";
  }
}

//////////////////////////////////////////////////
void World::InsertModelFile(const std::string &_sdfFilename)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf_filename(_sdfFilename);
  this->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelSDF(const sdf::SDF &_sdf)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdf.ToString());
  this->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelString(const std::string &_sdfString)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdfString);
  this->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
std::string World::StripWorldName(const std::string &_name) const
{
  if (_name.find(this->GetName() + "::") == 0)
    return _name.substr(this->GetName().size() + 2);
  else
    return _name;
}

//////////////////////////////////////////////////
void World::UpdateSDFFromState(const WorldState &_state)
{
  if (this->sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = this->sdf->GetElement("model");

    while (childElem)
    {
      for (unsigned int i = 0; i < _state.GetModelStateCount(); ++i)
      {
        ModelState modelState = _state.GetModelState(i);
        if (modelState.GetName() == childElem->GetValueString("name"))
        {
          modelState.UpdateModelSDF(childElem);
        }
      }

      childElem = childElem->GetNextElement("model");
    }
  }
}

//////////////////////////////////////////////////
void World::EnableAllModels()
{
  for (unsigned int i = 0; i < this->GetModelCount(); ++i)
    this->GetModel(i)->SetEnabled(true);
}

//////////////////////////////////////////////////
void World::DisableAllModels()
{
  for (unsigned int i = 0; i < this->GetModelCount(); ++i)
    this->GetModel(i)->SetEnabled(false);
}
