/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <time.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <sdf/sdf.hh>

#include <deque>
#include <list>
#include <set>
#include <string>
#include <vector>

#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/util/LogPlay.hh"

#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Plugin.hh"

#include "gazebo/math/Vector3.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/util/OpenAL.hh"
#include "gazebo/util/Diagnostics.hh"
#include "gazebo/util/LogRecord.hh"

#include "gazebo/physics/Road.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/common/SphericalCoordinates.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Population.hh"

using namespace gazebo;
using namespace physics;

/// \brief Flag used to say if/when to clear all models.
/// This will be replaced with a class member variable in Gazebo 3.0
bool g_clearModels;

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
  g_clearModels = false;
  this->sdf.reset(new sdf::Element);
  sdf::initFile("world.sdf", this->sdf);

  this->factorySDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->factorySDF);

  this->logPlayStateSDF.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->logPlayStateSDF);

  this->receiveMutex = new boost::recursive_mutex();
  this->loadModelMutex = new boost::mutex();

  this->initialized = false;
  this->loaded = false;
  this->stepInc = 0;
  this->pause = false;
  this->thread = NULL;
  this->logThread = NULL;
  this->stop = false;

  this->currentStateBuffer = 0;
  this->stateToggle = 0;

  this->pluginsLoaded = false;

  this->name = _name;

  this->needsReset = false;
  this->resetAll = true;
  this->resetTimeOnly = false;
  this->resetModelOnly = false;
  this->enablePhysicsEngine = true;
  this->setWorldPoseMutex = new boost::mutex();
  this->worldUpdateMutex = new boost::recursive_mutex();

  this->sleepOffset = common::Time(0);

  this->prevStatTime = common::Time::GetWallTime();
  this->prevProcessMsgsTime = common::Time::GetWallTime();

  this->connections.push_back(
     event::Events::ConnectStep(boost::bind(&World::OnStep, this)));
  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&World::SetSelectedEntityCB, this, _1)));
  this->connections.push_back(
     event::Events::ConnectPause(
       boost::bind(&World::SetPaused, this, _1)));
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
  this->loaded = false;
  this->sdf = _sdf;

  if (this->sdf->Get<std::string>("name").empty())
    gzwarn << "create_world(world_name =["
           << this->name << "]) overwrites sdf world name\n!";
  else
    this->name = this->sdf->Get<std::string>("name");

#ifdef HAVE_OPENAL
  util::OpenAL::Instance()->Load(this->sdf->GetElement("audio"));
#endif

  this->sceneMsg.CopyFrom(msgs::SceneFromSDF(this->sdf->GetElement("scene")));
  this->sceneMsg.set_name(this->GetName());

  // The period at which messages are processed
  this->processMsgsPeriod = common::Time(0, 200000000);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->GetName());

  // pose pub for server side, mainly used for updating and timestamping
  // Scene, which in turn will be used by rendering sensors.
  // TODO: replace local communication with shared memory for efficiency.
  this->poseLocalPub = this->node->Advertise<msgs::PosesStamped>(
    "~/pose/local/info", 10);

  // pose pub for client with a cap on publishing rate to reduce traffic
  // overhead
  this->posePub = this->node->Advertise<msgs::PosesStamped>(
    "~/pose/info", 10, 60);

  this->guiPub = this->node->Advertise<msgs::GUI>("~/gui", 5);
  if (this->sdf->HasElement("gui"))
    this->guiPub->Publish(msgs::GUIFromSDF(this->sdf->GetElement("gui")));

  this->factorySub = this->node->Subscribe("~/factory",
                                           &World::OnFactoryMsg, this);
  this->controlSub = this->node->Subscribe("~/world_control",
                                           &World::OnControl, this);

  this->requestSub = this->node->Subscribe("~/request",
                                           &World::OnRequest, this, true);
  this->jointSub = this->node->Subscribe("~/joint", &World::JointLog, this);
  this->lightSub = this->node->Subscribe("~/light", &World::OnLightMsg, this);

  this->modelSub = this->node->Subscribe<msgs::Model>("~/model/modify",
      &World::OnModelMsg, this);

  this->responsePub = this->node->Advertise<msgs::Response>("~/response");
  this->statPub =
    this->node->Advertise<msgs::WorldStatistics>("~/world_stats", 100, 5);
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection", 1);
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/info");
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  std::string type = this->sdf->GetElement("physics")->Get<std::string>("type");
  this->physicsEngine = PhysicsFactory::NewPhysicsEngine(type,
      shared_from_this());

  if (this->physicsEngine == NULL)
    gzthrow("Unable to create physics engine\n");

  // This should come before loading of entities
  this->physicsEngine->Load(this->sdf->GetElement("physics"));

  // This should also come before loading of entities
  {
    sdf::ElementPtr spherical = this->sdf->GetElement("spherical_coordinates");
    common::SphericalCoordinates::SurfaceType surfaceType =
      common::SphericalCoordinates::Convert(
        spherical->Get<std::string>("surface_model"));
    math::Angle latitude, longitude, heading;
    double elevation = spherical->Get<double>("elevation");
    latitude.SetFromDegree(spherical->Get<double>("latitude_deg"));
    longitude.SetFromDegree(spherical->Get<double>("longitude_deg"));
    heading.SetFromDegree(spherical->Get<double>("heading_deg"));

    this->sphericalCoordinates.reset(new common::SphericalCoordinates(
      surfaceType, latitude, longitude, elevation, heading));
  }

  if (this->sphericalCoordinates == NULL)
    gzthrow("Unable to create spherical coordinates data structure\n");

  this->rootElement.reset(new Base(BasePtr()));
  this->rootElement->SetName(this->GetName());
  this->rootElement->SetWorld(shared_from_this());

  // A special order is necessary when loading a world that contains state
  // information. The joints must be created last, otherwise they get
  // initialized improperly.
  {
    // Create all the entities
    this->LoadEntities(this->sdf, this->rootElement);

    // Set the state of the entities
    if (this->sdf->HasElement("state"))
    {
      sdf::ElementPtr childElem = this->sdf->GetElement("state");

      while (childElem)
      {
        WorldState myState;
        myState.Load(childElem);
        this->SetState(myState);

        childElem = childElem->GetNextElement("state");

        // TODO: We currently load just the first state data. Need to
        // implement a better mechanism for handling multiple states
        break;
      }
    }

    for (unsigned int i = 0; i < this->GetModelCount(); ++i)
      this->GetModel(i)->LoadJoints();
  }

  // TODO: Performance test to see if TBB model updating is necessary
  // Choose threaded or unthreaded model updating depending on the number of
  // models in the scene
  // if (this->GetModelCount() < 20)
  this->modelUpdateFunc = &World::ModelUpdateSingleLoop;
  // else
  // this->modelUpdateFunc = &World::ModelUpdateTBB;

  event::Events::worldCreated(this->GetName());

  this->loaded = true;
}

//////////////////////////////////////////////////
void World::Save(const std::string &_filename)
{
  this->UpdateStateSDF();
  std::string data;
  data = "<?xml version ='1.0'?>\n";
  data += "<sdf version='" +
          boost::lexical_cast<std::string>(SDF_VERSION) + "'>\n";
  data += this->sdf->ToString("");
  data += "</sdf>\n";

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
  // Initialize all the entities (i.e. Model)
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Init();

  // Initialize the physics engine
  this->physicsEngine->Init();

  this->testRay = boost::dynamic_pointer_cast<RayShape>(
      this->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  this->prevStates[0].SetWorld(shared_from_this());
  this->prevStates[1].SetWorld(shared_from_this());

  this->prevStates[0].SetName(this->GetName());
  this->prevStates[1].SetName(this->GetName());

  this->updateInfo.worldName = this->GetName();

  this->iterations = 0;
  this->logPrevIteration = 0;

  util::DiagnosticManager::Instance()->Init(this->GetName());

  util::LogRecord::Instance()->Add(this->GetName(), "state.log",
      boost::bind(&World::OnLog, this, _1));

  // Check if we have to insert an object population.
  if (this->sdf->HasElement("population"))
  {
    Population population(this->sdf, shared_from_this());
    population.PopulateAll();
  }

  this->initialized = true;

  // Mark the world initialization
  gzlog << "Init world[" << this->GetName() << "]" << std::endl;
}

//////////////////////////////////////////////////
void World::Run(unsigned int _iterations)
{
  this->stop = false;
  this->stopIterations = _iterations;

  this->thread = new boost::thread(boost::bind(&World::RunLoop, this));
}

//////////////////////////////////////////////////
void World::RunBlocking(unsigned int _iterations)
{
  this->stop = false;
  this->stopIterations = _iterations;
  this->RunLoop();
}

//////////////////////////////////////////////////
void World::RemoveModel(ModelPtr _model)
{
  if (_model)
    this->RemoveModel(_model->GetName());
}

//////////////////////////////////////////////////
bool World::GetRunning() const
{
  return !this->stop;
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

  // Get the first state
  this->prevStates[0] = WorldState(shared_from_this());
  this->prevStates[1] = WorldState(shared_from_this());
  this->stateToggle = 0;

  this->logThread = new boost::thread(boost::bind(&World::LogWorker, this));

  if (!util::LogPlay::Instance()->IsOpen())
  {
    for (this->iterations = 0; !this->stop &&
        (!this->stopIterations || (this->iterations < this->stopIterations));)
    {
      this->Step();
    }
  }
  else
  {
    this->enablePhysicsEngine = false;
    for (this->iterations = 0; !this->stop &&
        (!this->stopIterations || (this->iterations < this->stopIterations));)
    {
      this->LogStep();
    }
  }

  this->stop = true;

  if (this->logThread)
  {
    this->logCondition.notify_all();
    {
      boost::mutex::scoped_lock lock(this->logMutex);
      this->logCondition.notify_all();
    }
    this->logThread->join();
    delete this->logThread;
    this->logThread = NULL;
  }
}

//////////////////////////////////////////////////
void World::LogStep()
{
  if (!this->IsPaused() || this->stepInc > 0)
  {
    std::string data;
    if (!util::LogPlay::Instance()->Step(data))
    {
      this->SetPaused(true);
    }
    else
    {
      this->logPlayStateSDF->ClearElements();
      sdf::readString(data, this->logPlayStateSDF);

      this->logPlayState.Load(this->logPlayStateSDF);

      // Process insertions
      if (this->logPlayStateSDF->HasElement("insertions"))
      {
        sdf::ElementPtr modelElem =
          this->logPlayStateSDF->GetElement("insertions")->GetElement("model");

        while (modelElem)
        {
          ModelPtr model = this->LoadModel(modelElem, this->rootElement);
          model->Init();

          // Disabling plugins on playback
          // model->LoadPlugins();

          modelElem = modelElem->GetNextElement("model");
        }
      }

      // Process deletions
      if (this->logPlayStateSDF->HasElement("deletions"))
      {
        sdf::ElementPtr nameElem =
          this->logPlayStateSDF->GetElement("deletions")->GetElement("name");
        while (nameElem)
        {
          transport::requestNoReply(this->GetName(), "entity_delete",
                                    nameElem->Get<std::string>());
          nameElem = nameElem->GetNextElement("name");
        }
      }

      this->SetState(this->logPlayState);
      this->Update();
      this->iterations++;
    }

    if (this->stepInc > 0)
      this->stepInc--;
  }

  this->PublishWorldStats();

  this->ProcessMessages();
}

//////////////////////////////////////////////////
void World::Step()
{
  DIAG_TIMER_START("World::Step");

  /// need this because ODE does not call dxReallocateWorldProcessContext()
  /// until dWorld.*Step
  /// Plugins that manipulate joints (and probably other properties) require
  /// one iteration of the physics engine. Do not remove this.
  if (!this->pluginsLoaded &&
      sensors::SensorManager::Instance()->SensorsInitialized())
  {
    this->LoadPlugins();
    this->pluginsLoaded = true;
  }

  DIAG_TIMER_LAP("World::Step", "loadPlugins");

  // Send statistics about the world simulation
  this->PublishWorldStats();

  DIAG_TIMER_LAP("World::Step", "publishWorldStats");

  double updatePeriod = this->physicsEngine->GetUpdatePeriod();
  // sleep here to get the correct update rate
  common::Time tmpTime = common::Time::GetWallTime();
  common::Time sleepTime = this->prevStepWallTime +
    common::Time(updatePeriod) - tmpTime - this->sleepOffset;

  common::Time actualSleep = 0;
  if (sleepTime > 0)
  {
    common::Time::Sleep(sleepTime);
    actualSleep = common::Time::GetWallTime() - tmpTime;
  }
  else
    sleepTime = 0;

  // exponentially avg out
  this->sleepOffset = (actualSleep - sleepTime) * 0.01 +
                      this->sleepOffset * 0.99;

  DIAG_TIMER_LAP("World::Step", "sleepOffset");

  // throttling update rate, with sleepOffset as tolerance
  // the tolerance is needed as the sleep time is not exact
  if (common::Time::GetWallTime() - this->prevStepWallTime + this->sleepOffset
         >= common::Time(updatePeriod))
  {
    boost::recursive_mutex::scoped_lock lock(*this->worldUpdateMutex);

    DIAG_TIMER_LAP("World::Step", "worldUpdateMutex");

    this->prevStepWallTime = common::Time::GetWallTime();

    double stepTime = this->physicsEngine->GetMaxStepSize();
    if (!this->IsPaused() || this->stepInc > 0)
    {
      // query timestep to allow dynamic time step size updates
      this->simTime += stepTime;
      this->iterations++;
      this->Update();

      DIAG_TIMER_LAP("World::Step", "update");

      if (this->IsPaused() && this->stepInc > 0)
        this->stepInc--;
    }
    else
    {
      // Flush the log record buffer, if there is data in it.
      if (util::LogRecord::Instance()->GetBufferSize() > 0)
        util::LogRecord::Instance()->Notify();
      this->pauseTime += stepTime;
    }
  }

  this->ProcessMessages();

  DIAG_TIMER_STOP("World::Step");

  if (g_clearModels)
    this->ClearModels();
}

//////////////////////////////////////////////////
void World::Step(unsigned int _steps)
{
  if (!this->IsPaused())
  {
    gzwarn << "Calling World::Step(steps) while world is not paused\n";
    this->SetPaused(true);
  }

  {
    boost::recursive_mutex::scoped_lock lock(*this->worldUpdateMutex);
    this->stepInc = _steps;
  }

  // block on completion
  bool wait = true;
  while (wait)
  {
    common::Time::MSleep(1);
    boost::recursive_mutex::scoped_lock lock(*this->worldUpdateMutex);
    if (this->stepInc == 0 || this->stop)
      wait = false;
  }
}

//////////////////////////////////////////////////
void World::Update()
{
  DIAG_TIMER_START("World::Update");

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
  DIAG_TIMER_LAP("World::Update", "needsReset");

  this->updateInfo.simTime = this->GetSimTime();
  this->updateInfo.realTime = this->GetRealTime();
  event::Events::worldUpdateBegin(this->updateInfo);

  DIAG_TIMER_LAP("World::Update", "Events::worldUpdateBegin");

  // Update all the models
  (*this.*modelUpdateFunc)();

  DIAG_TIMER_LAP("World::Update", "Model::Update");

  // This must be called before PhysicsEngine::UpdatePhysics.
  this->physicsEngine->UpdateCollision();

  DIAG_TIMER_LAP("World::Update", "PhysicsEngine::UpdateCollision");

  // Wait for logging to finish, if it's running.
  if (util::LogRecord::Instance()->GetRunning())
  {
    boost::mutex::scoped_lock lock(this->logMutex);

    // It's possible the logWorker thread never processed the previous
    // state. This checks to make sure that we don't continute until the log
    // worker catchs up.
    if (this->iterations - this->logPrevIteration > 1)
    {
      this->logCondition.notify_one();
      this->logContinueCondition.wait(lock);
    }
  }

  // Update the physics engine
  if (this->enablePhysicsEngine && this->physicsEngine)
  {
    // This must be called directly after PhysicsEngine::UpdateCollision.
    this->physicsEngine->UpdatePhysics();

    DIAG_TIMER_LAP("World::Update", "PhysicsEngine::UpdatePhysics");

    // do this after physics update as
    //   ode --> MoveCallback sets the dirtyPoses
    //           and we need to propagate it into Entity::worldPose
    {
      // block any other pose updates (e.g. Joint::SetPosition)
      boost::recursive_mutex::scoped_lock lock(
        *this->physicsEngine->GetPhysicsUpdateMutex());

      for (std::list<Entity*>::iterator iter = this->dirtyPoses.begin();
          iter != this->dirtyPoses.end(); ++iter)
      {
        (*iter)->SetWorldPose((*iter)->GetDirtyPose(), false);
      }

      this->dirtyPoses.clear();
    }

    DIAG_TIMER_LAP("World::Update", "SetWorldPose(dirtyPoses)");
  }

  // Only update state information if logging data.
  if (util::LogRecord::Instance()->GetRunning())
    this->logCondition.notify_one();
  DIAG_TIMER_LAP("World::Update", "LogRecordNotify");

  // Output the contact information
  this->physicsEngine->GetContactManager()->PublishContacts();

  DIAG_TIMER_LAP("World::Update", "ContactManager::PublishContacts");

  event::Events::worldUpdateEnd();

  DIAG_TIMER_STOP("World::Update");
}

//////////////////////////////////////////////////
void World::Fini()
{
  this->Stop();
  this->plugins.clear();

  this->publishModelPoses.clear();

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

  this->models.clear();
  this->prevStates[0].SetWorld(WorldPtr());
  this->prevStates[1].SetWorld(WorldPtr());

#ifdef HAVE_OPENAL
  util::OpenAL::Instance()->Fini();
#endif
}

//////////////////////////////////////////////////
void World::Clear()
{
  g_clearModels = true;
}

//////////////////////////////////////////////////
void World::ClearModels()
{
  g_clearModels = false;
  bool pauseState = this->IsPaused();
  this->SetPaused(true);

  this->publishModelPoses.clear();

  // Remove all models
  for (Model_V::iterator iter = this->models.begin();
       iter != this->models.end(); ++iter)
  {
    this->rootElement->RemoveChild((*iter)->GetId());
  }
  this->models.clear();

  this->SetPaused(pauseState);
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
common::SphericalCoordinatesPtr World::GetSphericalCoordinates() const
{
  return this->sphericalCoordinates;
}

//////////////////////////////////////////////////
BasePtr World::GetByName(const std::string &_name)
{
  if (this->rootElement)
    return this->rootElement->GetByName(_name);
  else
    return BasePtr();
}

/////////////////////////////////////////////////
ModelPtr World::GetModelById(unsigned int _id)
{
  return boost::dynamic_pointer_cast<Model>(this->rootElement->GetById(_id));
}

//////////////////////////////////////////////////
ModelPtr World::GetModel(const std::string &_name)
{
  boost::mutex::scoped_lock lock(*this->loadModelMutex);
  return boost::dynamic_pointer_cast<Model>(this->GetByName(_name));
}

//////////////////////////////////////////////////
EntityPtr World::GetEntity(const std::string &_name)
{
  return boost::dynamic_pointer_cast<Entity>(this->GetByName(_name));
}

//////////////////////////////////////////////////
ModelPtr World::LoadModel(sdf::ElementPtr _sdf , BasePtr _parent)
{
  boost::mutex::scoped_lock lock(*this->loadModelMutex);
  ModelPtr model;

  if (_sdf->GetName() == "model")
  {
    model = this->physicsEngine->CreateModel(_parent);
    model->SetWorld(shared_from_this());
    model->Load(_sdf);

    event::Events::addEntity(model->GetScopedName());

    msgs::Model msg;
    model->FillMsg(msg);
    this->modelPub->Publish(msg);

    this->EnableAllModels();
  }
  else
  {
    gzerr << "SDF is missing the <model> tag:\n";
  }

  this->PublishModelPose(model);
  this->models.push_back(model);
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
  actor->FillMsg(msg);
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
  return this->models.size();
}

//////////////////////////////////////////////////
ModelPtr World::GetModel(unsigned int _index) const
{
  if (_index >= this->models.size())
  {
    gzerr << "Given model index[" << _index << "] is out of range[0.."
          << this->models.size() << "]\n";
    return ModelPtr();
  }

  return this->models[_index];
}

//////////////////////////////////////////////////
Model_V World::GetModels() const
{
  return this->models;
}

//////////////////////////////////////////////////
void World::ResetTime()
{
  this->simTime = common::Time(0);
  this->pauseTime = common::Time(0);
  this->startTime = common::Time::GetWallTime();
  this->realTimeOffset = common::Time(0);
  this->iterations = 0;
  sensors::SensorManager::Instance()->ResetLastUpdateTimes();
}

//////////////////////////////////////////////////
void World::ResetEntities(Base::EntityType _type)
{
  this->rootElement->Reset(_type);
}

//////////////////////////////////////////////////
void World::Reset()
{
  bool currentlyPaused = this->IsPaused();
  this->SetPaused(true);

  {
    boost::recursive_mutex::scoped_lock(*this->worldUpdateMutex);

    math::Rand::SetSeed(math::Rand::GetSeed());
    this->physicsEngine->SetSeed(math::Rand::GetSeed());

    this->ResetTime();
    this->ResetEntities(Base::BASE);
    for (std::vector<WorldPluginPtr>::iterator iter = this->plugins.begin();
        iter != this->plugins.end(); ++iter)
      (*iter)->Reset();
    this->physicsEngine->Reset();

    // Signal a reset has occurred
    event::Events::worldReset();
  }

  this->SetPaused(currentlyPaused);
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
  EntityPtr ent = boost::dynamic_pointer_cast<Entity>(base);

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
void World::SetSimTime(const common::Time &_t)
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
  if (!util::LogPlay::Instance()->IsOpen())
  {
    if (this->pause)
      return (this->pauseStartTime - this->startTime) - this->realTimeOffset;
    else
      return (common::Time::GetWallTime() - this->startTime) -
        this->realTimeOffset;
  }
  else
    return this->logRealTime;
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

  {
    boost::recursive_mutex::scoped_lock(*this->worldUpdateMutex);
    this->pause = _p;
  }

  if (_p)
  {
    // This is also a good time to clear out the logging buffer.
    util::LogRecord::Instance()->Notify();

    this->pauseStartTime = common::Time::GetWallTime();
  }
  else
    this->realTimeOffset += common::Time::GetWallTime() - this->pauseStartTime;

  event::Events::pause(_p);
}

//////////////////////////////////////////////////
void World::OnFactoryMsg(ConstFactoryPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
  this->factoryMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::OnControl(ConstWorldControlPtr &_data)
{
  if (_data->has_pause())
    this->SetPaused(_data->pause());

  if (_data->has_step())
    this->OnStep();

  if (_data->has_multi_step())
  {
    // stepWorld is a blocking call so set stepInc directly so that world stats
    // will still be published
    this->SetPaused(true);
    boost::recursive_mutex::scoped_lock lock(*this->worldUpdateMutex);
    this->stepInc = _data->multi_step();
  }

  if (_data->has_seed())
  {
    math::Rand::SetSeed(_data->seed());
    this->physicsEngine->SetSeed(_data->seed());
  }

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
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
  this->requestMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::JointLog(ConstJointPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
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
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
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
      boost::static_pointer_cast<Model>(_entity)->FillMsg(*modelMsg);
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
  tbb::parallel_for (tbb::blocked_range<size_t>(0, this->models.size(), 10),
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

  // Load the plugins for all the models
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
  {
    if (this->rootElement->GetChild(i)->HasType(Base::MODEL))
    {
      ModelPtr model = boost::static_pointer_cast<Model>(
          this->rootElement->GetChild(i));
      model->LoadPlugins();
    }
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
  std::string pluginName = _sdf->Get<std::string>("name");
  std::string filename = _sdf->Get<std::string>("filename");
  this->LoadPlugin(filename, pluginName, _sdf);
}

//////////////////////////////////////////////////
void World::ProcessEntityMsgs()
{
  boost::mutex::scoped_lock lock(this->entityDeleteMutex);

  std::list<std::string>::iterator iter;
  for (iter = this->deleteEntity.begin();
       iter != this->deleteEntity.end(); ++iter)
  {
    this->RemoveModel(*iter);
  }

  if (!this->deleteEntity.empty())
  {
    this->EnableAllModels();
    this->deleteEntity.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessRequestMsgs()
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
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
          ModelPtr model = boost::dynamic_pointer_cast<Model>(entity);
          model->FillMsg(*modelMsg);
        }
      }

      response.set_type(modelVMsg.GetTypeName());
      std::string *serializedData = response.mutable_serialized_data();
      modelVMsg.SerializeToString(serializedData);
    }
    else if ((*iter).request() == "entity_delete")
    {
      boost::mutex::scoped_lock lock2(this->entityDeleteMutex);
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
          ModelPtr model = boost::dynamic_pointer_cast<Model>(entity);
          model->FillMsg(modelMsg);

          std::string *serializedData = response.mutable_serialized_data();
          modelMsg.SerializeToString(serializedData);
          response.set_type(modelMsg.GetTypeName());
        }
        else if (entity->HasType(Base::LINK))
        {
          msgs::Link linkMsg;
          LinkPtr link = boost::dynamic_pointer_cast<Link>(entity);
          link->FillMsg(linkMsg);

          std::string *serializedData = response.mutable_serialized_data();
          linkMsg.SerializeToString(serializedData);
          response.set_type(linkMsg.GetTypeName());
        }
        else if (entity->HasType(Base::COLLISION))
        {
          msgs::Collision collisionMsg;
          CollisionPtr collision =
            boost::dynamic_pointer_cast<Collision>(entity);
          collision->FillMsg(collisionMsg);

          std::string *serializedData = response.mutable_serialized_data();
          collisionMsg.SerializeToString(serializedData);
          response.set_type(collisionMsg.GetTypeName());
        }
        else if (entity->HasType(Base::JOINT))
        {
          msgs::Joint jointMsg;
          JointPtr joint = boost::dynamic_pointer_cast<Joint>(entity);
          joint->FillMsg(jointMsg);

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
      std::ostringstream stream;
      stream << "<?xml version='1.0'?>\n"
             << "<sdf version='" << SDF_VERSION << "'>\n"
             << this->sdf->ToString("")
             << "</sdf>";

      msg.set_data(stream.str());

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
    else if ((*iter).request() == "spherical_coordinates_info")
    {
      msgs::SphericalCoordinates sphereCoordMsg;
      msgs::Set(&sphereCoordMsg, *(this->sphericalCoordinates));

      std::string *serializedData = response.mutable_serialized_data();
      sphereCoordMsg.SerializeToString(serializedData);
      response.set_type(sphereCoordMsg.GetTypeName());
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
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
  std::list<msgs::Model>::iterator iter;
  for (iter = this->modelMsgs.begin(); iter != this->modelMsgs.end(); ++iter)
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

      // May 30, 2013: The following code was removed because it has a
      // major performance impact when dragging complex object via the GUI.
      // This code also does not seem to be necessary, since can just
      // publish the incoming changes instead of a full model message. We
      // are leaving it temporarily in case we find a need for it.
      //
      // Let all other subscribers know about the change
      // msgs::Model msg;
      // model->FillMsg(msg);
      // // FillMsg fills the visual components from initial sdf
      // // but problem is that Visuals may have changed e.g. through ~/visual,
      // // so don't publish them to subscribers.
      // for (int i = 0; i < msg.link_size(); ++i)
      // {
      //   msg.mutable_link(i)->clear_visual();
      //   for (int j = 0; j < msg.link(i).collision_size(); ++j)
      //   {
      //     msg.mutable_link(i)->mutable_collision(j)->clear_visual();
      //   }
      // }

      this->modelPub->Publish(*iter);
    }
  }

  if (!this->modelMsgs.empty())
  {
    this->EnableAllModels();
    this->modelMsgs.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessFactoryMsgs()
{
  std::list<sdf::ElementPtr> modelsToLoad;
  std::list<msgs::Factory>::iterator iter;

  {
    boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
    for (iter = this->factoryMsgs.begin();
        iter != this->factoryMsgs.end(); ++iter)
    {
      this->factorySDF->root->ClearElements();

      if ((*iter).has_sdf() && !(*iter).sdf().empty())
      {
        // SDF Parsing happens here
        if (!sdf::readString((*iter).sdf(), factorySDF))
        {
          gzerr << "Unable to read sdf string[" << (*iter).sdf() << "]\n";
          continue;
        }
      }
      else if ((*iter).has_sdf_filename() && !(*iter).sdf_filename().empty())
      {
        std::string filename = common::ModelDatabase::Instance()->GetModelFile(
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

        factorySDF->root->GetElement("model")->GetAttribute("name")->Set(
            newName);
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
          if (factorySDF->root->GetName() == "sdf")
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

        sdf::ElementPtr elem = factorySDF->root->Clone();

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
          modelsToLoad.push_back(elem);
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

  for (std::list<sdf::ElementPtr>::iterator iter2 = modelsToLoad.begin();
       iter2 != modelsToLoad.end(); ++iter2)
  {
    try
    {
      boost::mutex::scoped_lock lock(this->factoryDeleteMutex);

      ModelPtr model = this->LoadModel(*iter2, this->rootElement);
      model->Init();
      model->LoadPlugins();
    }
    catch(...)
    {
      gzerr << "Loading model from factory message failed\n";
    }
  }
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
void World::SetState(const WorldState &_state)
{
  this->SetSimTime(_state.GetSimTime());
  this->logRealTime = _state.GetRealTime();

  const ModelState_M modelStates = _state.GetModelStates();
  for (ModelState_M::const_iterator iter = modelStates.begin();
       iter != modelStates.end(); ++iter)
  {
    ModelPtr model = this->GetModel(iter->second.GetName());
    if (model)
      model->SetState(iter->second);
    else
      gzerr << "Unable to find model[" << iter->second.GetName() << "]\n";
  }
}

//////////////////////////////////////////////////
void World::InsertModelFile(const std::string &_sdfFilename)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf_filename(_sdfFilename);
  this->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelSDF(const sdf::SDF &_sdf)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdf.ToString());
  this->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelString(const std::string &_sdfString)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);
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
void World::EnableAllModels()
{
  for (Model_V::iterator iter = this->models.begin();
       iter != this->models.end(); ++iter)
  {
    (*iter)->SetEnabled(true);
  }
}

//////////////////////////////////////////////////
void World::DisableAllModels()
{
  for (Model_V::iterator iter = this->models.begin();
       iter != this->models.end(); ++iter)
  {
    (*iter)->SetEnabled(false);
  }
}

//////////////////////////////////////////////////
void World::UpdateStateSDF()
{
  this->sdf->Update();
  sdf::ElementPtr stateElem = this->sdf->GetElement("state");
  stateElem->ClearElements();

  WorldState currentState(shared_from_this());
  currentState.FillSDF(stateElem);
}

//////////////////////////////////////////////////
bool World::OnLog(std::ostringstream &_stream)
{
  int bufferIndex = this->currentStateBuffer;
  // Save the entire state when its the first call to OnLog.
  if (util::LogRecord::Instance()->GetFirstUpdate())
  {
    this->UpdateStateSDF();
    _stream << "<sdf version ='";
    _stream << SDF_VERSION;
    _stream << "'>\n";
    _stream << this->sdf->ToString("");
    _stream << "</sdf>\n";
  }
  else if (this->states[bufferIndex].size() >= 1)
  {
    {
      boost::mutex::scoped_lock lock(this->logBufferMutex);
      this->currentStateBuffer ^= 1;
    }
    for (std::deque<WorldState>::iterator iter =
        this->states[bufferIndex].begin();
        iter != this->states[bufferIndex].end(); ++iter)
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>" << *iter << "</sdf>";
    }

    this->states[bufferIndex].clear();
  }

  // Logging has stopped. Wait for log worker to finish. Output last bit
  // of data, and reset states.
  if (!util::LogRecord::Instance()->GetRunning())
  {
    boost::mutex::scoped_lock lock(this->logBufferMutex);

    // Output any data that may have been pushed onto the queue
    for (size_t i = 0; i < this->states[this->currentStateBuffer^1].size(); ++i)
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>"
        << this->states[this->currentStateBuffer^1][i] << "</sdf>";
    }
    for (size_t i = 0; i < this->states[this->currentStateBuffer].size(); ++i)
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>"
        << this->states[this->currentStateBuffer][i] << "</sdf>";
    }

    // Clear everything.
    this->states[0].clear();
    this->states[1].clear();
    this->stateToggle = 0;
    this->prevStates[0] = WorldState();
    this->prevStates[1] = WorldState();
  }

  return true;
}

//////////////////////////////////////////////////
void World::ProcessMessages()
{
  {
    boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);

    if ((this->posePub && this->posePub->HasConnections()) ||
        (this->poseLocalPub && this->poseLocalPub->HasConnections()))
    {
      msgs::PosesStamped msg;

      // Time stamp this PosesStamped message
      msgs::Set(msg.mutable_time(), this->GetSimTime());

      if (!this->publishModelPoses.empty())
      {
        for (std::set<ModelPtr>::iterator iter =
            this->publishModelPoses.begin();
            iter != this->publishModelPoses.end(); ++iter)
        {
          msgs::Pose *poseMsg = msg.add_pose();

          // Publish the model's relative pose
          poseMsg->set_name((*iter)->GetScopedName());
          poseMsg->set_id((*iter)->GetId());
          msgs::Set(poseMsg, (*iter)->GetRelativePose());

          // Publish each of the model's children relative poses
          Link_V links = (*iter)->GetLinks();
          for (Link_V::iterator linkIter = links.begin();
              linkIter != links.end(); ++linkIter)
          {
            poseMsg = msg.add_pose();
            poseMsg->set_name((*linkIter)->GetScopedName());
            poseMsg->set_id((*linkIter)->GetId());
            msgs::Set(poseMsg, (*linkIter)->GetRelativePose());
          }
        }

        if (this->posePub && this->posePub->HasConnections())
          this->posePub->Publish(msg);
      }

      if (this->poseLocalPub && this->poseLocalPub->HasConnections())
      {
        // rendering::Scene depends on this timestamp, which is used by
        // rendering sensors to time stamp their data
        this->poseLocalPub->Publish(msg);
      }
    }
    this->publishModelPoses.clear();
  }

  if (common::Time::GetWallTime() - this->prevProcessMsgsTime >
      this->processMsgsPeriod)
  {
    this->ProcessEntityMsgs();
    this->ProcessRequestMsgs();
    this->ProcessFactoryMsgs();
    this->ProcessModelMsgs();
    this->prevProcessMsgsTime = common::Time::GetWallTime();
  }
}

//////////////////////////////////////////////////
void World::PublishWorldStats()
{
  msgs::Set(this->worldStatsMsg.mutable_sim_time(), this->GetSimTime());
  msgs::Set(this->worldStatsMsg.mutable_real_time(), this->GetRealTime());
  msgs::Set(this->worldStatsMsg.mutable_pause_time(), this->GetPauseTime());
  this->worldStatsMsg.set_iterations(this->iterations);
  this->worldStatsMsg.set_paused(this->IsPaused());

  if (this->statPub && this->statPub->HasConnections())
    this->statPub->Publish(this->worldStatsMsg);
  this->prevStatTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
bool World::IsLoaded() const
{
  return this->loaded;
}

//////////////////////////////////////////////////
void World::PublishModelPose(physics::ModelPtr _model)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);

  // Only add if the model name is not in the list
  this->publishModelPoses.insert(_model);
}

//////////////////////////////////////////////////
void World::LogWorker()
{
  boost::mutex::scoped_lock lock(this->logMutex);

  WorldPtr self = shared_from_this();
  this->logPrevIteration = this->iterations;

  GZ_ASSERT(self, "Self pointer to World is invalid");

  while (!this->stop)
  {
    int currState = (this->stateToggle + 1) % 2;

    this->prevStates[currState].Load(self);
    WorldState diffState = this->prevStates[currState] -
      this->prevStates[this->stateToggle];
    this->logPrevIteration = this->iterations;

    if (!diffState.IsZero())
    {
      this->stateToggle = currState;
      {
        // Store the entire current state (instead of the diffState). A slow
        // moving link may never be captured if only diff state is recorded.
        boost::mutex::scoped_lock bLock(this->logBufferMutex);
        this->states[this->currentStateBuffer].push_back(
            this->prevStates[currState]);
        // Tell the logger to update, once the number of states exceeds 1000
        if (this->states[this->currentStateBuffer].size() > 1000)
          util::LogRecord::Instance()->Notify();
      }
    }

    this->logContinueCondition.notify_all();

    // Wait until there is work to be done.
    this->logCondition.wait(lock);
  }

  // Make sure nothing is blocked by this thread.
  this->logContinueCondition.notify_all();
}

/////////////////////////////////////////////////
uint32_t World::GetIterations() const
{
  return this->iterations;
}

//////////////////////////////////////////////////
void World::RemoveModel(const std::string &_name)
{
  boost::mutex::scoped_lock flock(this->factoryDeleteMutex);

  // Remove all the dirty poses from the delete entity.
  {
    for (std::list<Entity*>::iterator iter2 = this->dirtyPoses.begin();
        iter2 != this->dirtyPoses.end();)
    {
      if ((*iter2)->GetName() == _name ||
          ((*iter2)->GetParent() && (*iter2)->GetParent()->GetName() == _name))
      {
        this->dirtyPoses.erase(iter2++);
      }
      else
        ++iter2;
    }
  }

  if (this->sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = this->sdf->GetElement("model");
    while (childElem && childElem->Get<std::string>("name") != _name)
      childElem = childElem->GetNextElement("model");
    if (childElem)
      this->sdf->RemoveChild(childElem);
  }

  if (this->sdf->HasElement("light"))
  {
    sdf::ElementPtr childElem = this->sdf->GetElement("light");
    while (childElem && childElem->Get<std::string>("name") != _name)
      childElem = childElem->GetNextElement("light");
    if (childElem)
    {
      this->sdf->RemoveChild(childElem);
      // Find the light by name in the scene msg, and remove it.
      for (int i = 0; i < this->sceneMsg.light_size(); ++i)
      {
        if (this->sceneMsg.light(i).name() == _name)
        {
          this->sceneMsg.mutable_light()->SwapElements(i,
              this->sceneMsg.light_size()-1);
          this->sceneMsg.mutable_light()->RemoveLast();
          break;
        }
      }
    }
  }

  {
    boost::recursive_mutex::scoped_lock lock(
        *this->GetPhysicsEngine()->GetPhysicsUpdateMutex());

    this->rootElement->RemoveChild(_name);

    for (Model_V::iterator iter = this->models.begin();
        iter != this->models.end(); ++iter)
    {
      if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
      {
        this->models.erase(iter);
        break;
      }
    }
  }

  // Cleanup the publishModelPoses list.
  {
    boost::recursive_mutex::scoped_lock lock2(*this->receiveMutex);
    for (std::set<ModelPtr>::iterator iter = this->publishModelPoses.begin();
        iter != this->publishModelPoses.end(); ++iter)
    {
      if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
      {
        this->publishModelPoses.erase(iter);
        break;
      }
    }
  }
}

/////////////////////////////////////////////////
void World::OnLightMsg(ConstLightPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->receiveMutex);

  bool lightExists = false;

  // Find the light by name, and copy the new parameters.
  for (int i = 0; i < this->sceneMsg.light_size(); ++i)
  {
    if (this->sceneMsg.light(i).name() == _msg->name())
    {
      lightExists = true;
      this->sceneMsg.mutable_light(i)->MergeFrom(*_msg);

      sdf::ElementPtr childElem = this->sdf->GetElement("light");
      while (childElem && childElem->Get<std::string>("name") != _msg->name())
        childElem = childElem->GetNextElement("light");
      if (childElem)
        msgs::LightToSDF(*_msg, childElem);
      break;
    }
  }

  // Add a new light if the light doesn't exist.
  if (!lightExists)
  {
    this->sceneMsg.add_light()->CopyFrom(*_msg);

    // add to the world sdf
    sdf::ElementPtr lightSDF = msgs::LightToSDF(*_msg);
    lightSDF->SetParent(this->sdf);
    lightSDF->GetParent()->InsertElement(lightSDF);
  }
}

/////////////////////////////////////////////////
msgs::Scene World::GetSceneMsg() const
{
  return this->sceneMsg;
}
