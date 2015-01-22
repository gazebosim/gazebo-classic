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
#include "gazebo/physics/WorldPrivate.hh"
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
  : dataPtr(new WorldPrivate)
{
  g_clearModels = false;
  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("world.sdf", this->dataPtr->sdf);

  this->dataPtr->factorySDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->dataPtr->factorySDF);

  this->dataPtr->logPlayStateSDF.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->dataPtr->logPlayStateSDF);

  this->dataPtr->receiveMutex = new boost::recursive_mutex();
  this->dataPtr->loadModelMutex = new boost::mutex();

  this->dataPtr->initialized = false;
  this->dataPtr->loaded = false;
  this->dataPtr->stepInc = 0;
  this->dataPtr->pause = false;
  this->dataPtr->thread = NULL;
  this->dataPtr->logThread = NULL;
  this->dataPtr->stop = false;

  this->dataPtr->currentStateBuffer = 0;
  this->dataPtr->stateToggle = 0;

  this->dataPtr->pluginsLoaded = false;

  this->dataPtr->name = _name;

  this->dataPtr->needsReset = false;
  this->dataPtr->resetAll = true;
  this->dataPtr->resetTimeOnly = false;
  this->dataPtr->resetModelOnly = false;
  this->dataPtr->enablePhysicsEngine = true;
  this->dataPtr->setWorldPoseMutex = new boost::mutex();
  this->dataPtr->worldUpdateMutex = new boost::recursive_mutex();

  this->dataPtr->sleepOffset = common::Time(0);

  this->dataPtr->prevStatTime = common::Time::GetWallTime();
  this->dataPtr->prevProcessMsgsTime = common::Time::GetWallTime();

  this->dataPtr->connections.push_back(
     event::Events::ConnectStep(boost::bind(&World::OnStep, this)));
  this->dataPtr->connections.push_back(
     event::Events::ConnectPause(
       boost::bind(&World::SetPaused, this, _1)));
}

//////////////////////////////////////////////////
World::~World()
{
  delete this->dataPtr->receiveMutex;
  this->dataPtr->receiveMutex = NULL;
  delete this->dataPtr->loadModelMutex;
  this->dataPtr->loadModelMutex = NULL;
  delete this->dataPtr->setWorldPoseMutex;
  this->dataPtr->setWorldPoseMutex = NULL;
  delete this->dataPtr->worldUpdateMutex;
  this->dataPtr->worldUpdateMutex = NULL;

  this->dataPtr->connections.clear();
  this->Fini();

  this->dataPtr->sdf->Reset();
  this->dataPtr->rootElement.reset();
  this->dataPtr->node.reset();

  this->dataPtr->testRay.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void World::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->loaded = false;
  this->dataPtr->sdf = _sdf;

  if (this->dataPtr->sdf->Get<std::string>("name").empty())
    gzwarn << "create_world(world_name =["
           << this->dataPtr->name << "]) overwrites sdf world name\n!";
  else
    this->dataPtr->name = this->dataPtr->sdf->Get<std::string>("name");

#ifdef HAVE_OPENAL
  util::OpenAL::Instance()->Load(this->dataPtr->sdf->GetElement("audio"));
#endif

  this->dataPtr->sceneMsg.CopyFrom(
      msgs::SceneFromSDF(this->dataPtr->sdf->GetElement("scene")));
  this->dataPtr->sceneMsg.set_name(this->GetName());

  // The period at which messages are processed
  this->dataPtr->processMsgsPeriod = common::Time(0, 200000000);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->GetName());

  // pose pub for server side, mainly used for updating and timestamping
  // Scene, which in turn will be used by rendering sensors.
  // TODO: replace local communication with shared memory for efficiency.
  this->dataPtr->poseLocalPub =
    this->dataPtr->node->Advertise<msgs::PosesStamped>("~/pose/local/info", 10);

  // pose pub for client with a cap on publishing rate to reduce traffic
  // overhead
  this->dataPtr->posePub = this->dataPtr->node->Advertise<msgs::PosesStamped>(
    "~/pose/info", 10, 60);

  this->dataPtr->guiPub = this->dataPtr->node->Advertise<msgs::GUI>("~/gui", 5);
  if (this->dataPtr->sdf->HasElement("gui"))
  {
    this->dataPtr->guiPub->Publish(
        msgs::GUIFromSDF(this->dataPtr->sdf->GetElement("gui")));
  }

  this->dataPtr->factorySub = this->dataPtr->node->Subscribe("~/factory",
                                           &World::OnFactoryMsg, this);
  this->dataPtr->controlSub = this->dataPtr->node->Subscribe("~/world_control",
                                           &World::OnControl, this);

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
                                           &World::OnRequest, this, true);
  this->dataPtr->jointSub = this->dataPtr->node->Subscribe("~/joint",
      &World::JointLog, this);
  this->dataPtr->lightSub = this->dataPtr->node->Subscribe("~/light",
      &World::OnLightMsg, this);

  this->dataPtr->modelSub = this->dataPtr->node->Subscribe<msgs::Model>(
      "~/model/modify", &World::OnModelMsg, this);

  this->dataPtr->responsePub = this->dataPtr->node->Advertise<msgs::Response>(
      "~/response");
  this->dataPtr->statPub =
    this->dataPtr->node->Advertise<msgs::WorldStatistics>(
        "~/world_stats", 100, 5);
  this->dataPtr->modelPub = this->dataPtr->node->Advertise<msgs::Model>(
      "~/model/info");
  this->dataPtr->lightPub = this->dataPtr->node->Advertise<msgs::Light>(
      "~/light");

  std::string type = this->dataPtr->sdf->GetElement(
      "physics")->Get<std::string>("type");
  this->dataPtr->physicsEngine = PhysicsFactory::NewPhysicsEngine(type,
      shared_from_this());

  if (this->dataPtr->physicsEngine == NULL)
    gzthrow("Unable to create physics engine\n");

  // This should come before loading of entities
  this->dataPtr->physicsEngine->Load(this->dataPtr->sdf->GetElement("physics"));

  this->dataPtr->presetManager =
    new PresetManager(WorldPtr(this), this->dataPtr->sdf->GetElement("physics"));

  // This should also come before loading of entities
  {
    sdf::ElementPtr spherical = this->dataPtr->sdf->GetElement(
        "spherical_coordinates");
    common::SphericalCoordinates::SurfaceType surfaceType =
      common::SphericalCoordinates::Convert(
        spherical->Get<std::string>("surface_model"));
    math::Angle latitude, longitude, heading;
    double elevation = spherical->Get<double>("elevation");
    latitude.SetFromDegree(spherical->Get<double>("latitude_deg"));
    longitude.SetFromDegree(spherical->Get<double>("longitude_deg"));
    heading.SetFromDegree(spherical->Get<double>("heading_deg"));

    this->dataPtr->sphericalCoordinates.reset(new common::SphericalCoordinates(
      surfaceType, latitude, longitude, elevation, heading));
  }

  if (this->dataPtr->sphericalCoordinates == NULL)
    gzthrow("Unable to create spherical coordinates data structure\n");

  this->dataPtr->rootElement.reset(new Base(BasePtr()));
  this->dataPtr->rootElement->SetName(this->GetName());
  this->dataPtr->rootElement->SetWorld(shared_from_this());

  // A special order is necessary when loading a world that contains state
  // information. The joints must be created last, otherwise they get
  // initialized improperly.
  {
    // Create all the entities
    this->LoadEntities(this->dataPtr->sdf, this->dataPtr->rootElement);

    // Set the state of the entities
    if (this->dataPtr->sdf->HasElement("state"))
    {
      sdf::ElementPtr childElem = this->dataPtr->sdf->GetElement("state");

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
  this->dataPtr->modelUpdateFunc = &World::ModelUpdateSingleLoop;
  // else
  // this->dataPtr->modelUpdateFunc = &World::ModelUpdateTBB;

  event::Events::worldCreated(this->GetName());

  this->dataPtr->loaded = true;
}

//////////////////////////////////////////////////
void World::Save(const std::string &_filename)
{
  this->UpdateStateSDF();
  std::string data;
  data = "<?xml version ='1.0'?>\n";
  data += "<sdf version='" +
          boost::lexical_cast<std::string>(SDF_VERSION) + "'>\n";
  data += this->dataPtr->sdf->ToString("");
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
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); i++)
    this->dataPtr->rootElement->GetChild(i)->Init();

  // Initialize the physics engine
  this->dataPtr->physicsEngine->Init();

  this->dataPtr->testRay = boost::dynamic_pointer_cast<RayShape>(
      this->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  this->dataPtr->prevStates[0].SetWorld(shared_from_this());
  this->dataPtr->prevStates[1].SetWorld(shared_from_this());

  this->dataPtr->prevStates[0].SetName(this->GetName());
  this->dataPtr->prevStates[1].SetName(this->GetName());

  this->dataPtr->updateInfo.worldName = this->GetName();

  this->dataPtr->iterations = 0;
  this->dataPtr->logPrevIteration = 0;

  util::DiagnosticManager::Instance()->Init(this->GetName());

  util::LogRecord::Instance()->Add(this->GetName(), "state.log",
      boost::bind(&World::OnLog, this, _1));

  // Check if we have to insert an object population.
  if (this->dataPtr->sdf->HasElement("population"))
  {
    Population population(this->dataPtr->sdf, shared_from_this());
    population.PopulateAll();
  }

  this->dataPtr->initialized = true;

  // Mark the world initialization
  gzlog << "Init world[" << this->GetName() << "]" << std::endl;
}

//////////////////////////////////////////////////
void World::Run(unsigned int _iterations)
{
  this->dataPtr->stop = false;
  this->dataPtr->stopIterations = _iterations;

  this->dataPtr->thread = new boost::thread(boost::bind(&World::RunLoop, this));
}

//////////////////////////////////////////////////
void World::RunBlocking(unsigned int _iterations)
{
  this->dataPtr->stop = false;
  this->dataPtr->stopIterations = _iterations;
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
  return !this->dataPtr->stop;
}

//////////////////////////////////////////////////
void World::Stop()
{
  this->dataPtr->stop = true;

  if (this->dataPtr->thread)
  {
    this->dataPtr->thread->join();
    delete this->dataPtr->thread;
    this->dataPtr->thread = NULL;
  }
}

//////////////////////////////////////////////////
void World::RunLoop()
{
  this->dataPtr->physicsEngine->InitForThread();

  this->dataPtr->startTime = common::Time::GetWallTime();

  // This fixes a minor issue when the world is paused before it's started
  if (this->IsPaused())
    this->dataPtr->pauseStartTime = this->dataPtr->startTime;

  this->dataPtr->prevStepWallTime = common::Time::GetWallTime();

  // Get the first state
  this->dataPtr->prevStates[0] = WorldState(shared_from_this());
  this->dataPtr->prevStates[1] = WorldState(shared_from_this());
  this->dataPtr->stateToggle = 0;

  this->dataPtr->logThread =
    new boost::thread(boost::bind(&World::LogWorker, this));

  if (!util::LogPlay::Instance()->IsOpen())
  {
    for (this->dataPtr->iterations = 0; !this->dataPtr->stop &&
        (!this->dataPtr->stopIterations ||
         (this->dataPtr->iterations < this->dataPtr->stopIterations));)
    {
      this->Step();
    }
  }
  else
  {
    this->dataPtr->enablePhysicsEngine = false;
    for (this->dataPtr->iterations = 0; !this->dataPtr->stop &&
        (!this->dataPtr->stopIterations ||
         (this->dataPtr->iterations < this->dataPtr->stopIterations));)
    {
      this->LogStep();
    }
  }

  this->dataPtr->stop = true;

  if (this->dataPtr->logThread)
  {
    this->dataPtr->logCondition.notify_all();
    {
      boost::mutex::scoped_lock lock(this->dataPtr->logMutex);
      this->dataPtr->logCondition.notify_all();
    }
    this->dataPtr->logThread->join();
    delete this->dataPtr->logThread;
    this->dataPtr->logThread = NULL;
  }
}

//////////////////////////////////////////////////
void World::LogStep()
{
  if (!this->IsPaused() || this->dataPtr->stepInc > 0)
  {
    std::string data;
    if (!util::LogPlay::Instance()->Step(data))
    {
      this->SetPaused(true);
    }
    else
    {
      this->dataPtr->logPlayStateSDF->ClearElements();
      sdf::readString(data, this->dataPtr->logPlayStateSDF);

      this->dataPtr->logPlayState.Load(this->dataPtr->logPlayStateSDF);

      // Process insertions
      if (this->dataPtr->logPlayStateSDF->HasElement("insertions"))
      {
        sdf::ElementPtr modelElem =
          this->dataPtr->logPlayStateSDF->GetElement(
              "insertions")->GetElement("model");

        while (modelElem)
        {
          ModelPtr model = this->LoadModel(modelElem,
              this->dataPtr->rootElement);
          model->Init();

          // Disabling plugins on playback
          // model->LoadPlugins();

          modelElem = modelElem->GetNextElement("model");
        }
      }

      // Process deletions
      if (this->dataPtr->logPlayStateSDF->HasElement("deletions"))
      {
        sdf::ElementPtr nameElem =
          this->dataPtr->logPlayStateSDF->GetElement(
              "deletions")->GetElement("name");

        while (nameElem)
        {
          transport::requestNoReply(this->GetName(), "entity_delete",
                                    nameElem->Get<std::string>());
          nameElem = nameElem->GetNextElement("name");
        }
      }

      this->SetState(this->dataPtr->logPlayState);
      this->Update();
      this->dataPtr->iterations++;
    }

    if (this->dataPtr->stepInc > 0)
      this->dataPtr->stepInc--;
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
  if (!this->dataPtr->pluginsLoaded &&
      sensors::SensorManager::Instance()->SensorsInitialized())
  {
    this->LoadPlugins();
    this->dataPtr->pluginsLoaded = true;
  }

  DIAG_TIMER_LAP("World::Step", "loadPlugins");

  // Send statistics about the world simulation
  this->PublishWorldStats();

  DIAG_TIMER_LAP("World::Step", "publishWorldStats");

  double updatePeriod = this->dataPtr->physicsEngine->GetUpdatePeriod();
  // sleep here to get the correct update rate
  common::Time tmpTime = common::Time::GetWallTime();
  common::Time sleepTime = this->dataPtr->prevStepWallTime +
    common::Time(updatePeriod) - tmpTime - this->dataPtr->sleepOffset;

  common::Time actualSleep = 0;
  if (sleepTime > 0)
  {
    common::Time::Sleep(sleepTime);
    actualSleep = common::Time::GetWallTime() - tmpTime;
  }
  else
    sleepTime = 0;

  // exponentially avg out
  this->dataPtr->sleepOffset = (actualSleep - sleepTime) * 0.01 +
                      this->dataPtr->sleepOffset * 0.99;

  DIAG_TIMER_LAP("World::Step", "sleepOffset");

  // throttling update rate, with sleepOffset as tolerance
  // the tolerance is needed as the sleep time is not exact
  if (common::Time::GetWallTime() - this->dataPtr->prevStepWallTime +
      this->dataPtr->sleepOffset >= common::Time(updatePeriod))
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->worldUpdateMutex);

    DIAG_TIMER_LAP("World::Step", "worldUpdateMutex");

    this->dataPtr->prevStepWallTime = common::Time::GetWallTime();

    double stepTime = this->dataPtr->physicsEngine->GetMaxStepSize();
    if (!this->IsPaused() || this->dataPtr->stepInc > 0)
    {
      // query timestep to allow dynamic time step size updates
      this->dataPtr->simTime += stepTime;
      this->dataPtr->iterations++;
      this->Update();

      DIAG_TIMER_LAP("World::Step", "update");

      if (this->IsPaused() && this->dataPtr->stepInc > 0)
        this->dataPtr->stepInc--;
    }
    else
    {
      // Flush the log record buffer, if there is data in it.
      if (util::LogRecord::Instance()->GetBufferSize() > 0)
        util::LogRecord::Instance()->Notify();
      this->dataPtr->pauseTime += stepTime;
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
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->worldUpdateMutex);
    this->dataPtr->stepInc = _steps;
  }

  // block on completion
  bool wait = true;
  while (wait)
  {
    common::Time::MSleep(1);
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->worldUpdateMutex);
    if (this->dataPtr->stepInc == 0 || this->dataPtr->stop)
      wait = false;
  }
}

//////////////////////////////////////////////////
void World::Update()
{
  DIAG_TIMER_START("World::Update");

  if (this->dataPtr->needsReset)
  {
    if (this->dataPtr->resetAll)
      this->Reset();
    else if (this->dataPtr->resetTimeOnly)
      this->ResetTime();
    else if (this->dataPtr->resetModelOnly)
      this->ResetEntities(Base::MODEL);
    this->dataPtr->needsReset = false;
  }
  DIAG_TIMER_LAP("World::Update", "needsReset");

  this->dataPtr->updateInfo.simTime = this->GetSimTime();
  this->dataPtr->updateInfo.realTime = this->GetRealTime();
  event::Events::worldUpdateBegin(this->dataPtr->updateInfo);

  DIAG_TIMER_LAP("World::Update", "Events::worldUpdateBegin");

  // Update all the models
  (*this.*dataPtr->modelUpdateFunc)();

  DIAG_TIMER_LAP("World::Update", "Model::Update");

  // This must be called before PhysicsEngine::UpdatePhysics.
  this->dataPtr->physicsEngine->UpdateCollision();

  DIAG_TIMER_LAP("World::Update", "PhysicsEngine::UpdateCollision");

  // Wait for logging to finish, if it's running.
  if (util::LogRecord::Instance()->GetRunning())
  {
    boost::mutex::scoped_lock lock(this->dataPtr->logMutex);

    // It's possible the logWorker thread never processed the previous
    // state. This checks to make sure that we don't continute until the log
    // worker catchs up.
    if (this->dataPtr->iterations - this->dataPtr->logPrevIteration > 1)
    {
      this->dataPtr->logCondition.notify_one();
      this->dataPtr->logContinueCondition.wait(lock);
    }
  }

  // Update the physics engine
  if (this->dataPtr->enablePhysicsEngine && this->dataPtr->physicsEngine)
  {
    // This must be called directly after PhysicsEngine::UpdateCollision.
    this->dataPtr->physicsEngine->UpdatePhysics();

    DIAG_TIMER_LAP("World::Update", "PhysicsEngine::UpdatePhysics");

    // do this after physics update as
    //   ode --> MoveCallback sets the dirtyPoses
    //           and we need to propagate it into Entity::worldPose
    {
      // block any other pose updates (e.g. Joint::SetPosition)
      boost::recursive_mutex::scoped_lock lock(
        *this->dataPtr->physicsEngine->GetPhysicsUpdateMutex());

      for (std::list<Entity*>::iterator iter =
          this->dataPtr->dirtyPoses.begin();
          iter != this->dataPtr->dirtyPoses.end(); ++iter)
      {
        (*iter)->SetWorldPose((*iter)->GetDirtyPose(), false);
      }

      this->dataPtr->dirtyPoses.clear();
    }

    DIAG_TIMER_LAP("World::Update", "SetWorldPose(dirtyPoses)");
  }

  // Only update state information if logging data.
  if (util::LogRecord::Instance()->GetRunning())
    this->dataPtr->logCondition.notify_one();
  DIAG_TIMER_LAP("World::Update", "LogRecordNotify");

  // Output the contact information
  this->dataPtr->physicsEngine->GetContactManager()->PublishContacts();

  DIAG_TIMER_LAP("World::Update", "ContactManager::PublishContacts");

  event::Events::worldUpdateEnd();

  DIAG_TIMER_STOP("World::Update");
}

//////////////////////////////////////////////////
void World::Fini()
{
  this->Stop();
  this->dataPtr->plugins.clear();

  this->dataPtr->publishModelPoses.clear();

  this->dataPtr->node->Fini();

  if (this->dataPtr->rootElement)
  {
    this->dataPtr->rootElement->Fini();
    this->dataPtr->rootElement.reset();
  }

  if (this->dataPtr->physicsEngine)
  {
    this->dataPtr->physicsEngine->Fini();
    this->dataPtr->physicsEngine.reset();
  }

  this->dataPtr->models.clear();
  this->dataPtr->prevStates[0].SetWorld(WorldPtr());
  this->dataPtr->prevStates[1].SetWorld(WorldPtr());

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

  this->dataPtr->publishModelPoses.clear();

  // Remove all models
  for (Model_V::iterator iter = this->dataPtr->models.begin();
       iter != this->dataPtr->models.end(); ++iter)
  {
    this->dataPtr->rootElement->RemoveChild((*iter)->GetId());
  }
  this->dataPtr->models.clear();

  this->SetPaused(pauseState);
}

//////////////////////////////////////////////////
std::string World::GetName() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
PhysicsEnginePtr World::GetPhysicsEngine() const
{
  return this->dataPtr->physicsEngine;
}

//////////////////////////////////////////////////
common::SphericalCoordinatesPtr World::GetSphericalCoordinates() const
{
  return this->dataPtr->sphericalCoordinates;
}

//////////////////////////////////////////////////
BasePtr World::GetByName(const std::string &_name)
{
  if (this->dataPtr->rootElement)
    return this->dataPtr->rootElement->GetByName(_name);
  else
    return BasePtr();
}

/////////////////////////////////////////////////
ModelPtr World::GetModelById(unsigned int _id)
{
  return boost::dynamic_pointer_cast<Model>(
      this->dataPtr->rootElement->GetById(_id));
}

//////////////////////////////////////////////////
ModelPtr World::GetModel(const std::string &_name)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->loadModelMutex);
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
  boost::mutex::scoped_lock lock(*this->dataPtr->loadModelMutex);
  ModelPtr model;

  if (_sdf->GetName() == "model")
  {
    model = this->dataPtr->physicsEngine->CreateModel(_parent);
    model->SetWorld(shared_from_this());
    model->Load(_sdf);

    event::Events::addEntity(model->GetScopedName());

    msgs::Model msg;
    model->FillMsg(msg);
    this->dataPtr->modelPub->Publish(msg);

    this->EnableAllModels();
  }
  else
  {
    gzerr << "SDF is missing the <model> tag:\n";
  }

  this->PublishModelPose(model);
  this->dataPtr->models.push_back(model);
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
  this->dataPtr->modelPub->Publish(msg);

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
      msgs::Light *lm = this->dataPtr->sceneMsg.add_light();
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
  return this->dataPtr->models.size();
}

//////////////////////////////////////////////////
ModelPtr World::GetModel(unsigned int _index) const
{
  if (_index >= this->dataPtr->models.size())
  {
    gzerr << "Given model index[" << _index << "] is out of range[0.."
          << this->dataPtr->models.size() << "]\n";
    return ModelPtr();
  }

  return this->dataPtr->models[_index];
}

//////////////////////////////////////////////////
Model_V World::GetModels() const
{
  return this->dataPtr->models;
}

//////////////////////////////////////////////////
void World::ResetTime()
{
  this->dataPtr->simTime = common::Time(0);
  this->dataPtr->pauseTime = common::Time(0);
  this->dataPtr->startTime = common::Time::GetWallTime();
  this->dataPtr->realTimeOffset = common::Time(0);
  this->dataPtr->iterations = 0;
  sensors::SensorManager::Instance()->ResetLastUpdateTimes();
}

//////////////////////////////////////////////////
void World::ResetEntities(Base::EntityType _type)
{
  this->dataPtr->rootElement->Reset(_type);
}

//////////////////////////////////////////////////
void World::Reset()
{
  bool currentlyPaused = this->IsPaused();
  this->SetPaused(true);

  {
    boost::recursive_mutex::scoped_lock(*this->dataPtr->worldUpdateMutex);

    math::Rand::SetSeed(math::Rand::GetSeed());
    this->dataPtr->physicsEngine->SetSeed(math::Rand::GetSeed());

    this->ResetTime();
    this->ResetEntities(Base::BASE);
    for (std::vector<WorldPluginPtr>::iterator iter =
        this->dataPtr->plugins.begin();
        iter != this->dataPtr->plugins.end(); ++iter)
    {
      (*iter)->Reset();
    }
    this->dataPtr->physicsEngine->Reset();

    // Signal a reset has occurred
    event::Events::worldReset();
  }

  this->SetPaused(currentlyPaused);
}

//////////////////////////////////////////////////
void World::OnStep()
{
  this->dataPtr->stepInc = 1;
}

//////////////////////////////////////////////////
void World::PrintEntityTree()
{
  // Initialize all the entities
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); i++)
    this->dataPtr->rootElement->GetChild(i)->Print("");
}

//////////////////////////////////////////////////
gazebo::common::Time World::GetSimTime() const
{
  return this->dataPtr->simTime;
}

//////////////////////////////////////////////////
void World::SetSimTime(const common::Time &_t)
{
  this->dataPtr->simTime = _t;
}

//////////////////////////////////////////////////
gazebo::common::Time World::GetPauseTime() const
{
  return this->dataPtr->pauseTime;
}

//////////////////////////////////////////////////
gazebo::common::Time World::GetStartTime() const
{
  return this->dataPtr->startTime;
}

//////////////////////////////////////////////////
common::Time World::GetRealTime() const
{
  if (!util::LogPlay::Instance()->IsOpen())
  {
    if (this->dataPtr->pause)
    {
      return (this->dataPtr->pauseStartTime - this->dataPtr->startTime) -
        this->dataPtr->realTimeOffset;
    }
    else
    {
      return (common::Time::GetWallTime() - this->dataPtr->startTime) -
        this->dataPtr->realTimeOffset;
    }
  }
  else
    return this->dataPtr->logRealTime;
}

//////////////////////////////////////////////////
bool World::IsPaused() const
{
  return this->dataPtr->pause;
}

//////////////////////////////////////////////////
void World::SetPaused(bool _p)
{
  if (this->dataPtr->pause == _p)
    return;

  {
    boost::recursive_mutex::scoped_lock(*this->dataPtr->worldUpdateMutex);
    this->dataPtr->pause = _p;
  }

  if (_p)
  {
    // This is also a good time to clear out the logging buffer.
    util::LogRecord::Instance()->Notify();

    this->dataPtr->pauseStartTime = common::Time::GetWallTime();
  }
  else
  {
    this->dataPtr->realTimeOffset += common::Time::GetWallTime() -
      this->dataPtr->pauseStartTime;
  }

  event::Events::pause(_p);
}

//////////////////////////////////////////////////
void World::OnFactoryMsg(ConstFactoryPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->factoryMsgs.push_back(*_msg);
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
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->worldUpdateMutex);
    this->dataPtr->stepInc = _data->multi_step();
  }

  if (_data->has_seed())
  {
    math::Rand::SetSeed(_data->seed());
    this->dataPtr->physicsEngine->SetSeed(_data->seed());
  }

  if (_data->has_reset())
  {
    this->dataPtr->needsReset = true;

    if (_data->reset().has_all() && _data->reset().all())
    {
      this->dataPtr->resetAll = true;
    }
    else
    {
      this->dataPtr->resetAll = false;

      if (_data->reset().has_time_only() && _data->reset().time_only())
        this->dataPtr->resetTimeOnly = true;

      if (_data->reset().has_model_only() && _data->reset().model_only())
        this->dataPtr->resetModelOnly = true;
    }
  }
}

//////////////////////////////////////////////////
void World::OnRequest(ConstRequestPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->requestMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::JointLog(ConstJointPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  int i = 0;
  for (; i < this->dataPtr->sceneMsg.joint_size(); i++)
  {
    if (this->dataPtr->sceneMsg.joint(i).name() == _msg->name())
    {
      this->dataPtr->sceneMsg.mutable_joint(i)->CopyFrom(*_msg);
      break;
    }
  }

  if (i >= this->dataPtr->sceneMsg.joint_size())
  {
    msgs::Joint *newJoint = this->dataPtr->sceneMsg.add_joint();
    newJoint->CopyFrom(*_msg);
  }
}

//////////////////////////////////////////////////
void World::OnModelMsg(ConstModelPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->modelMsgs.push_back(*_msg);
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
// void World::ModelUpdateTBB()
// {
//   tbb::parallel_for (tbb::blocked_range<size_t>(0,
//   this->dataPtr->models.size(), 10),
//       ModelUpdate_TBB(&this->dataPtr->models));
// }

//////////////////////////////////////////////////
void World::ModelUpdateSingleLoop()
{
  // Update all the models
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); i++)
    this->dataPtr->rootElement->GetChild(i)->Update();
}


//////////////////////////////////////////////////
void World::LoadPlugins()
{
  // Load the plugins
  if (this->dataPtr->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->dataPtr->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  // Load the plugins for all the models
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); i++)
  {
    if (this->dataPtr->rootElement->GetChild(i)->HasType(Base::MODEL))
    {
      ModelPtr model = boost::static_pointer_cast<Model>(
          this->dataPtr->rootElement->GetChild(i));
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
    this->dataPtr->plugins.push_back(plugin);

    if (this->dataPtr->initialized)
      plugin->Init();
  }
}

//////////////////////////////////////////////////
void World::RemovePlugin(const std::string &_name)
{
  std::vector<WorldPluginPtr>::iterator iter;
  for (iter = this->dataPtr->plugins.begin();
      iter != this->dataPtr->plugins.end(); ++iter)
  {
    if ((*iter)->GetHandle() == _name)
    {
      this->dataPtr->plugins.erase(iter);
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
  boost::mutex::scoped_lock lock(this->dataPtr->entityDeleteMutex);

  std::list<std::string>::iterator iter;
  for (iter = this->dataPtr->deleteEntity.begin();
       iter != this->dataPtr->deleteEntity.end(); ++iter)
  {
    this->RemoveModel(*iter);
  }

  if (!this->dataPtr->deleteEntity.empty())
  {
    this->EnableAllModels();
    this->dataPtr->deleteEntity.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessRequestMsgs()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  msgs::Response response;

  std::list<msgs::Request>::iterator iter;
  for (iter = this->dataPtr->requestMsgs.begin();
       iter != this->dataPtr->requestMsgs.end(); ++iter)
  {
    bool send = true;
    response.set_id((*iter).id());
    response.set_request((*iter).request());
    response.set_response("success");

    if ((*iter).request() == "entity_list")
    {
      msgs::Model_V modelVMsg;

      for (unsigned int i = 0;
          i < this->dataPtr->rootElement->GetChildCount(); ++i)
      {
        BasePtr entity = this->dataPtr->rootElement->GetChild(i);
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
      boost::mutex::scoped_lock lock2(this->dataPtr->entityDeleteMutex);
      this->dataPtr->deleteEntity.push_back((*iter).data());
    }
    else if ((*iter).request() == "entity_info")
    {
      BasePtr entity = this->dataPtr->rootElement->GetByName((*iter).data());
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
             << this->dataPtr->sdf->ToString("")
             << "</sdf>";

      msg.set_data(stream.str());

      std::string *serializedData = response.mutable_serialized_data();
      msg.SerializeToString(serializedData);
      response.set_type(msg.GetTypeName());
    }
    else if ((*iter).request() == "scene_info")
    {
      this->dataPtr->sceneMsg.clear_model();
      this->BuildSceneMsg(this->dataPtr->sceneMsg, this->dataPtr->rootElement);

      std::string *serializedData = response.mutable_serialized_data();
      this->dataPtr->sceneMsg.SerializeToString(serializedData);
      response.set_type(this->dataPtr->sceneMsg.GetTypeName());
    }
    else if ((*iter).request() == "spherical_coordinates_info")
    {
      msgs::SphericalCoordinates sphereCoordMsg;
      msgs::Set(&sphereCoordMsg, *(this->dataPtr->sphericalCoordinates));

      std::string *serializedData = response.mutable_serialized_data();
      sphereCoordMsg.SerializeToString(serializedData);
      response.set_type(sphereCoordMsg.GetTypeName());
    }
    else
      send = false;

    if (send)
    {
      this->dataPtr->responsePub->Publish(response);
    }
  }

  this->dataPtr->requestMsgs.clear();
}

//////////////////////////////////////////////////
void World::ProcessModelMsgs()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  std::list<msgs::Model>::iterator iter;
  for (iter = this->dataPtr->modelMsgs.begin();
      iter != this->dataPtr->modelMsgs.end(); ++iter)
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

      this->dataPtr->modelPub->Publish(*iter);
    }
  }

  if (!this->dataPtr->modelMsgs.empty())
  {
    this->EnableAllModels();
    this->dataPtr->modelMsgs.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessFactoryMsgs()
{
  std::list<sdf::ElementPtr> modelsToLoad;
  std::list<msgs::Factory>::iterator iter;

  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
    for (iter = this->dataPtr->factoryMsgs.begin();
        iter != this->dataPtr->factoryMsgs.end(); ++iter)
    {
      this->dataPtr->factorySDF->root->ClearElements();

      if ((*iter).has_sdf() && !(*iter).sdf().empty())
      {
        // SDF Parsing happens here
        if (!sdf::readString((*iter).sdf(), this->dataPtr->factorySDF))
        {
          gzerr << "Unable to read sdf string[" << (*iter).sdf() << "]\n";
          continue;
        }
      }
      else if ((*iter).has_sdf_filename() && !(*iter).sdf_filename().empty())
      {
        std::string filename = common::ModelDatabase::Instance()->GetModelFile(
            (*iter).sdf_filename());

        if (!sdf::readFile(filename, this->dataPtr->factorySDF))
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

        this->dataPtr->factorySDF->root->InsertElement(
            model->GetSDF()->Clone());

        std::string newName = model->GetName() + "_clone";
        int i = 0;
        while (this->GetModel(newName))
        {
          newName = model->GetName() + "_clone_" +
            boost::lexical_cast<std::string>(i);
          i++;
        }

        this->dataPtr->factorySDF->root->GetElement("model")->GetAttribute(
            "name")->Set(newName);
      }
      else
      {
        gzerr << "Unable to load sdf from factory message."
          << "No SDF or SDF filename specified.\n";
        continue;
      }

      if ((*iter).has_edit_name())
      {
        BasePtr base =
          this->dataPtr->rootElement->GetByName((*iter).edit_name());
        if (base)
        {
          sdf::ElementPtr elem;
          if (this->dataPtr->factorySDF->root->GetName() == "sdf")
            elem = this->dataPtr->factorySDF->root->GetFirstElement();
          else
            elem = this->dataPtr->factorySDF->root;

          base->UpdateParameters(elem);
        }
      }
      else
      {
        bool isActor = false;
        bool isModel = false;
        bool isLight = false;

        sdf::ElementPtr elem = this->dataPtr->factorySDF->root->Clone();

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
          this->dataPtr->factorySDF->root->PrintValues("");
          continue;
        }

        if (!elem)
        {
          gzerr << "Invalid SDF:";
          this->dataPtr->factorySDF->root->PrintValues("");
          continue;
        }

        elem->SetParent(this->dataPtr->sdf);
        elem->GetParent()->InsertElement(elem);
        if ((*iter).has_pose())
          elem->GetElement("pose")->Set(msgs::Convert((*iter).pose()));

        if (isActor)
        {
          ActorPtr actor = this->LoadActor(elem, this->dataPtr->rootElement);
          actor->Init();
        }
        else if (isModel)
        {
          modelsToLoad.push_back(elem);
        }
        else if (isLight)
        {
          /// \TODO: Current broken. See Issue #67.
          msgs::Light *lm = this->dataPtr->sceneMsg.add_light();
          lm->CopyFrom(msgs::LightFromSDF(elem));

          this->dataPtr->lightPub->Publish(*lm);
        }
      }
    }

    this->dataPtr->factoryMsgs.clear();
  }

  for (std::list<sdf::ElementPtr>::iterator iter2 = modelsToLoad.begin();
       iter2 != modelsToLoad.end(); ++iter2)
  {
    try
    {
      boost::mutex::scoped_lock lock(this->dataPtr->factoryDeleteMutex);

      ModelPtr model = this->LoadModel(*iter2, this->dataPtr->rootElement);
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

  this->dataPtr->physicsEngine->InitForThread();
  this->dataPtr->testRay->SetPoints(_pt, end);
  this->dataPtr->testRay->GetIntersection(dist, entityName);
  return this->GetEntity(entityName);
}

//////////////////////////////////////////////////
void World::SetState(const WorldState &_state)
{
  this->SetSimTime(_state.GetSimTime());
  this->dataPtr->logRealTime = _state.GetRealTime();

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
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf_filename(_sdfFilename);
  this->dataPtr->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelSDF(const sdf::SDF &_sdf)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdf.ToString());
  this->dataPtr->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelString(const std::string &_sdfString)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdfString);
  this->dataPtr->factoryMsgs.push_back(msg);
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
  for (Model_V::iterator iter = this->dataPtr->models.begin();
       iter != this->dataPtr->models.end(); ++iter)
  {
    (*iter)->SetEnabled(true);
  }
}

//////////////////////////////////////////////////
void World::DisableAllModels()
{
  for (Model_V::iterator iter = this->dataPtr->models.begin();
       iter != this->dataPtr->models.end(); ++iter)
  {
    (*iter)->SetEnabled(false);
  }
}

//////////////////////////////////////////////////
void World::UpdateStateSDF()
{
  this->dataPtr->sdf->Update();
  sdf::ElementPtr stateElem = this->dataPtr->sdf->GetElement("state");
  stateElem->ClearElements();

  WorldState currentState(shared_from_this());
  currentState.FillSDF(stateElem);
}

//////////////////////////////////////////////////
bool World::OnLog(std::ostringstream &_stream)
{
  int bufferIndex = this->dataPtr->currentStateBuffer;
  // Save the entire state when its the first call to OnLog.
  if (util::LogRecord::Instance()->GetFirstUpdate())
  {
    this->UpdateStateSDF();
    _stream << "<sdf version ='";
    _stream << SDF_VERSION;
    _stream << "'>\n";
    _stream << this->dataPtr->sdf->ToString("");
    _stream << "</sdf>\n";
  }
  else if (this->dataPtr->states[bufferIndex].size() >= 1)
  {
    {
      boost::mutex::scoped_lock lock(this->dataPtr->logBufferMutex);
      this->dataPtr->currentStateBuffer ^= 1;
    }
    for (std::deque<WorldState>::iterator iter =
        this->dataPtr->states[bufferIndex].begin();
        iter != this->dataPtr->states[bufferIndex].end(); ++iter)
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>" << *iter << "</sdf>";
    }

    this->dataPtr->states[bufferIndex].clear();
  }

  // Logging has stopped. Wait for log worker to finish. Output last bit
  // of data, and reset states.
  if (!util::LogRecord::Instance()->GetRunning())
  {
    boost::mutex::scoped_lock lock(this->dataPtr->logBufferMutex);

    // Output any data that may have been pushed onto the queue
    for (size_t i = 0;
        i < this->dataPtr->states[this->dataPtr->currentStateBuffer^1].size();
        ++i)
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>"
        << this->dataPtr->states[this->dataPtr->currentStateBuffer^1][i]
        << "</sdf>";
    }

    for (size_t i = 0;
        i < this->dataPtr->states[this->dataPtr->currentStateBuffer].size();
        ++i)
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>"
        << this->dataPtr->states[this->dataPtr->currentStateBuffer][i]
        << "</sdf>";
    }

    // Clear everything.
    this->dataPtr->states[0].clear();
    this->dataPtr->states[1].clear();
    this->dataPtr->stateToggle = 0;
    this->dataPtr->prevStates[0] = WorldState();
    this->dataPtr->prevStates[1] = WorldState();
  }

  return true;
}

//////////////////////////////////////////////////
void World::ProcessMessages()
{
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);

    if ((this->dataPtr->posePub && this->dataPtr->posePub->HasConnections()) ||
        (this->dataPtr->poseLocalPub &&
         this->dataPtr->poseLocalPub->HasConnections()))
    {
      msgs::PosesStamped msg;

      // Time stamp this PosesStamped message
      msgs::Set(msg.mutable_time(), this->GetSimTime());

      if (!this->dataPtr->publishModelPoses.empty())
      {
        for (std::set<ModelPtr>::iterator iter =
            this->dataPtr->publishModelPoses.begin();
            iter != this->dataPtr->publishModelPoses.end(); ++iter)
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

        if (this->dataPtr->posePub && this->dataPtr->posePub->HasConnections())
          this->dataPtr->posePub->Publish(msg);
      }

      if (this->dataPtr->poseLocalPub &&
          this->dataPtr->poseLocalPub->HasConnections())
      {
        // rendering::Scene depends on this timestamp, which is used by
        // rendering sensors to time stamp their data
        this->dataPtr->poseLocalPub->Publish(msg);
      }
    }
    this->dataPtr->publishModelPoses.clear();
  }

  if (common::Time::GetWallTime() - this->dataPtr->prevProcessMsgsTime >
      this->dataPtr->processMsgsPeriod)
  {
    this->ProcessEntityMsgs();
    this->ProcessRequestMsgs();
    this->ProcessFactoryMsgs();
    this->ProcessModelMsgs();
    this->dataPtr->prevProcessMsgsTime = common::Time::GetWallTime();
  }
}

//////////////////////////////////////////////////
void World::PublishWorldStats()
{
  msgs::Set(this->dataPtr->worldStatsMsg.mutable_sim_time(),
      this->GetSimTime());
  msgs::Set(this->dataPtr->worldStatsMsg.mutable_real_time(),
      this->GetRealTime());
  msgs::Set(this->dataPtr->worldStatsMsg.mutable_pause_time(),
      this->GetPauseTime());

  this->dataPtr->worldStatsMsg.set_iterations(this->dataPtr->iterations);
  this->dataPtr->worldStatsMsg.set_paused(this->IsPaused());

  if (this->dataPtr->statPub && this->dataPtr->statPub->HasConnections())
    this->dataPtr->statPub->Publish(this->dataPtr->worldStatsMsg);
  this->dataPtr->prevStatTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
bool World::IsLoaded() const
{
  return this->dataPtr->loaded;
}

//////////////////////////////////////////////////
void World::PublishModelPose(physics::ModelPtr _model)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);

  // Only add if the model name is not in the list
  this->dataPtr->publishModelPoses.insert(_model);
}

//////////////////////////////////////////////////
void World::LogWorker()
{
  boost::mutex::scoped_lock lock(this->dataPtr->logMutex);

  WorldPtr self = shared_from_this();
  this->dataPtr->logPrevIteration = this->dataPtr->iterations;

  GZ_ASSERT(self, "Self pointer to World is invalid");

  while (!this->dataPtr->stop)
  {
    int currState = (this->dataPtr->stateToggle + 1) % 2;

    this->dataPtr->prevStates[currState].Load(self);
    WorldState diffState = this->dataPtr->prevStates[currState] -
      this->dataPtr->prevStates[this->dataPtr->stateToggle];
    this->dataPtr->logPrevIteration = this->dataPtr->iterations;

    if (!diffState.IsZero())
    {
      this->dataPtr->stateToggle = currState;
      {
        // Store the entire current state (instead of the diffState). A slow
        // moving link may never be captured if only diff state is recorded.
        boost::mutex::scoped_lock bLock(this->dataPtr->logBufferMutex);
        this->dataPtr->states[this->dataPtr->currentStateBuffer].push_back(
            this->dataPtr->prevStates[currState]);
        // Tell the logger to update, once the number of states exceeds 1000
        if (this->dataPtr->states[this->dataPtr->currentStateBuffer].size() >
            1000)
        {
          util::LogRecord::Instance()->Notify();
        }
      }
    }

    this->dataPtr->logContinueCondition.notify_all();

    // Wait until there is work to be done.
    this->dataPtr->logCondition.wait(lock);
  }

  // Make sure nothing is blocked by this thread.
  this->dataPtr->logContinueCondition.notify_all();
}

/////////////////////////////////////////////////
uint32_t World::GetIterations() const
{
  return this->dataPtr->iterations;
}

//////////////////////////////////////////////////
void World::RemoveModel(const std::string &_name)
{
  boost::mutex::scoped_lock flock(this->dataPtr->factoryDeleteMutex);

  // Remove all the dirty poses from the delete entity.
  {
    for (std::list<Entity*>::iterator iter2 = this->dataPtr->dirtyPoses.begin();
        iter2 != this->dataPtr->dirtyPoses.end();)
    {
      if ((*iter2)->GetName() == _name ||
          ((*iter2)->GetParent() && (*iter2)->GetParent()->GetName() == _name))
      {
        this->dataPtr->dirtyPoses.erase(iter2++);
      }
      else
        ++iter2;
    }
  }

  if (this->dataPtr->sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = this->dataPtr->sdf->GetElement("model");
    while (childElem && childElem->Get<std::string>("name") != _name)
      childElem = childElem->GetNextElement("model");
    if (childElem)
      this->dataPtr->sdf->RemoveChild(childElem);
  }

  if (this->dataPtr->sdf->HasElement("light"))
  {
    sdf::ElementPtr childElem = this->dataPtr->sdf->GetElement("light");
    while (childElem && childElem->Get<std::string>("name") != _name)
      childElem = childElem->GetNextElement("light");
    if (childElem)
    {
      this->dataPtr->sdf->RemoveChild(childElem);
      // Find the light by name in the scene msg, and remove it.
      for (int i = 0; i < this->dataPtr->sceneMsg.light_size(); ++i)
      {
        if (this->dataPtr->sceneMsg.light(i).name() == _name)
        {
          this->dataPtr->sceneMsg.mutable_light()->SwapElements(i,
              this->dataPtr->sceneMsg.light_size()-1);
          this->dataPtr->sceneMsg.mutable_light()->RemoveLast();
          break;
        }
      }
    }
  }

  {
    boost::recursive_mutex::scoped_lock lock(
        *this->GetPhysicsEngine()->GetPhysicsUpdateMutex());

    this->dataPtr->rootElement->RemoveChild(_name);

    for (Model_V::iterator iter = this->dataPtr->models.begin();
        iter != this->dataPtr->models.end(); ++iter)
    {
      if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
      {
        this->dataPtr->models.erase(iter);
        break;
      }
    }
  }

  // Cleanup the publishModelPoses list.
  {
    boost::recursive_mutex::scoped_lock lock2(*this->dataPtr->receiveMutex);
    for (std::set<ModelPtr>::iterator iter =
         this->dataPtr->publishModelPoses.begin();
         iter != this->dataPtr->publishModelPoses.end(); ++iter)
    {
      if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
      {
        this->dataPtr->publishModelPoses.erase(iter);
        break;
      }
    }
  }
}

/////////////////////////////////////////////////
void World::OnLightMsg(ConstLightPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->receiveMutex);

  bool lightExists = false;

  // Find the light by name, and copy the new parameters.
  for (int i = 0; i < this->dataPtr->sceneMsg.light_size(); ++i)
  {
    if (this->dataPtr->sceneMsg.light(i).name() == _msg->name())
    {
      lightExists = true;
      this->dataPtr->sceneMsg.mutable_light(i)->MergeFrom(*_msg);

      sdf::ElementPtr childElem = this->dataPtr->sdf->GetElement("light");
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
    this->dataPtr->sceneMsg.add_light()->CopyFrom(*_msg);

    // add to the world sdf
    sdf::ElementPtr lightSDF = msgs::LightToSDF(*_msg);
    lightSDF->SetParent(this->dataPtr->sdf);
    lightSDF->GetParent()->InsertElement(lightSDF);
  }
}

/////////////////////////////////////////////////
msgs::Scene World::GetSceneMsg() const
{
  return this->dataPtr->sceneMsg;
}

/////////////////////////////////////////////////
boost::mutex *World::GetSetWorldPoseMutex() const
{
  return this->dataPtr->setWorldPoseMutex;
}

/////////////////////////////////////////////////
bool World::GetEnablePhysicsEngine()
{
  return this->dataPtr->enablePhysicsEngine;
}

/////////////////////////////////////////////////
void World::EnablePhysicsEngine(bool _enable)
{
  this->dataPtr->enablePhysicsEngine = _enable;
}

