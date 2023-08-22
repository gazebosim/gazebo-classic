/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <sdf/sdf.hh>

#include <deque>
#include <list>
#include <set>
#include <string>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>
#include <ignition/math/Rand.hh>
#include <ignition/math/SemanticVersion.hh>

#include <gazebo/gazebo_config.h>

#include <ignition/msgs/plugin_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include "ignition/common/Profiler.hh"
#include "ignition/common/URI.hh"
#include "gazebo/common/FuelModelDatabase.hh"

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
#include "gazebo/common/SdfFrameSemantics.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/util/OpenAL.hh"
#include "gazebo/util/Diagnostics.hh"
#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/util/LogRecord.hh"

#include "gazebo/physics/Road.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/Atmosphere.hh"
#include "gazebo/physics/AtmosphereFactory.hh"
#include "gazebo/physics/PresetManager.hh"
#include "gazebo/physics/UserCmdManager.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/Wind.hh"
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
  public: explicit ModelUpdate_TBB(Model_V *_models) : models(_models) {}
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

  // Keep this in the constructor for performance.
  // sdf::initFile causes disk access.
  this->dataPtr->factorySDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->dataPtr->factorySDF);

  this->dataPtr->logPlayStateSDF.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->dataPtr->logPlayStateSDF);

  this->dataPtr->initialized = false;
  this->dataPtr->loaded = false;
  this->dataPtr->stepInc = 0;
  this->dataPtr->pause = false;
  this->dataPtr->thread = nullptr;
  this->dataPtr->logThread = nullptr;
  this->dataPtr->stop = false;
  this->dataPtr->sensorsInitialized = false;

  this->dataPtr->currentStateBuffer = 0;
  this->dataPtr->stateToggle = 0;

  this->dataPtr->pluginsLoaded = false;

  this->dataPtr->name = _name;

  this->dataPtr->needsReset = false;
  this->dataPtr->resetAll = true;
  this->dataPtr->resetTimeOnly = false;
  this->dataPtr->resetModelOnly = false;
  this->dataPtr->enablePhysicsEngine = true;
  this->dataPtr->enableWind = true;
  this->dataPtr->enableAtmosphere = true;

  this->dataPtr->sleepOffset = common::Time(0);

  this->dataPtr->prevStatTime = common::Time::GetWallTime();
  this->dataPtr->prevProcessMsgsTime = common::Time::GetWallTime();
  this->dataPtr->logLastStatePlayedSimTime = common::Time(0);
  this->dataPtr->logLastStatePlayedRealTime = common::Time(0);
  this->dataPtr->logPlayRealTimeFactor = 0.0;

  this->dataPtr->connections.push_back(
     event::Events::ConnectStep(std::bind(&World::OnStep, this)));
  this->dataPtr->connections.push_back(
     event::Events::ConnectPause(
       std::bind(&World::SetPaused, this, std::placeholders::_1)));

  this->dataPtr->waitForSensors = nullptr;

  // Make sure dbs are initialized
  common::ModelDatabase::Instance();
  common::FuelModelDatabase::Instance();
}

//////////////////////////////////////////////////
World::~World()
{
  this->Fini();
}

//////////////////////////////////////////////////
void World::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->loaded = false;
  this->dataPtr->sdf = _sdf;
  common::convertToFullPaths(this->dataPtr->sdf);

  // Create a DOM object to compute the resolved initial pose (with frame
  // semantics)
  this->dataPtr->worldSDFDom = std::make_unique<sdf::World>();
  sdf::Errors errors = this->dataPtr->worldSDFDom->Load(_sdf);

  // Print errors and load the parts that worked.
  auto sdfVersion =
    ignition::math::SemanticVersion(_sdf->OriginalVersion());
  // Only print out errors if the original SDFormat version does not support
  // frame semantics
  if (sdfVersion >= ignition::math::SemanticVersion(1, 7))
  {
    for (const auto &error : errors)
    {
      if (common::isSdfFrameSemanticsError(error))
      {
        gzerr << error << "\n";
      }
    }
  }

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
  this->dataPtr->sceneMsg.set_name(this->Name());

  if (this->dataPtr->sdf->GetElement("scene")->
      HasElement("ignition:shadow_caster_material_name"))
  {
    this->dataPtr->shadowCasterMaterialName =
      this->dataPtr->sdf->GetElement("scene")->
        Get<std::string>("ignition:shadow_caster_material_name");
  }

  if (this->dataPtr->sdf->GetElement("scene")->
      HasElement("ignition:shadow_caster_render_back_faces"))
  {
    this->dataPtr->shadowCasterRenderBackFaces =
      this->dataPtr->sdf->GetElement("scene")->
        Get<bool>("ignition:shadow_caster_render_back_faces");
  }

  {
    const std::string kElementName = "ignition:model_plugin_loading_timeout";
    if (this->dataPtr->sdf->HasElement(kElementName))
    {
      this->dataPtr->modelPluginLoadingTimeout =
        this->dataPtr->sdf->Get<unsigned int>(kElementName);
    }
  }

  // The period at which messages are processed
  this->dataPtr->processMsgsPeriod = common::Time(0, 200000000);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->Name());

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
  this->dataPtr->playbackControlSub = this->dataPtr->node->Subscribe(
      "~/playback_control", &World::OnPlaybackControl, this);

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
                                           &World::OnRequest, this, true);
  this->dataPtr->jointSub = this->dataPtr->node->Subscribe("~/joint",
      &World::JointLog, this);

  this->dataPtr->lightFactorySub =
      this->dataPtr->node->Subscribe("~/factory/light",
      &World::OnLightFactoryMsg, this);
  this->dataPtr->lightModifySub =
      this->dataPtr->node->Subscribe("~/light/modify",
      &World::OnLightModifyMsg, this);

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
      "~/light/modify");
  this->dataPtr->lightFactoryPub = this->dataPtr->node->Advertise<msgs::Light>(
      "~/factory/light");

  // Ignition transport
  std::string pluginInfoService("/physics/info/plugin");
  if (!this->dataPtr->ignNode.Advertise(pluginInfoService,
      &World::PluginInfoService, this))
  {
    gzerr << "Error advertising service [" << pluginInfoService << "]"
        << std::endl;
  }

  std::string sceneInfoService("/scene_info");
  if (!this->dataPtr->ignNode.Advertise(sceneInfoService,
      &World::SceneInfoService, this))
  {
    gzerr << "Error advertising service [" << sceneInfoService << "]"
        << std::endl;
  }

  std::string shadowCasterMaterialNameService("/shadow_caster_material_name");
  if (!this->dataPtr->ignNode.Advertise(shadowCasterMaterialNameService,
      &World::ShadowCasterMaterialNameService, this))
  {
    gzerr << "Error advertising service [" <<
        shadowCasterMaterialNameService << "]" << std::endl;
  }

  std::string shadowCasterRenderBackFacesService(
      "/shadow_caster_render_back_faces");
  if (!this->dataPtr->ignNode.Advertise(shadowCasterRenderBackFacesService,
      &World::ShadowCasterRenderBackFacesService, this))
  {
    gzerr << "Error advertising service [" <<
        shadowCasterRenderBackFacesService << "]" << std::endl;
  }

  std::string materialShininessService("/shininess");
  if (!this->dataPtr->ignNode.Advertise(materialShininessService,
      &World::MaterialShininessService, this))
  {
    gzerr << "Error advertising service ["
          << materialShininessService << "]" << std::endl;
  }

  // This should come before loading of entities
  sdf::ElementPtr physicsElem = this->dataPtr->sdf->GetElement("physics");

  std::string type = physicsElem->Get<std::string>("type");
  this->dataPtr->physicsEngine = PhysicsFactory::NewPhysicsEngine(type,
      shared_from_this());

  if (this->dataPtr->physicsEngine == nullptr)
    gzthrow("Unable to create physics engine\n");

  this->dataPtr->physicsEngine->Load(physicsElem);

  // This should come before loading of entities
  sdf::ElementPtr windElem = this->dataPtr->sdf->GetElement("wind");

  this->dataPtr->wind.reset(new physics::Wind(*this,
                            this->dataPtr->sdf->GetElement("wind")));

  if (this->dataPtr->wind == NULL)
    gzthrow("Unable to create wind\n");

  this->dataPtr->wind->Load(windElem);

  // This should come after loading physics engine
  sdf::ElementPtr atmosphereElem = this->dataPtr->sdf->GetElement("atmosphere");

  type = atmosphereElem->Get<std::string>("type");
  this->dataPtr->atmosphere = AtmosphereFactory::NewAtmosphere(type, *this);

  if (this->dataPtr->atmosphere == NULL)
    gzerr << "Unable to create atmosphere model\n";

  this->dataPtr->atmosphere->Load(atmosphereElem);

  // This should also come before loading of entities
  {
    sdf::ElementPtr spherical = this->dataPtr->sdf->GetElement(
        "spherical_coordinates");
    common::SphericalCoordinates::SurfaceType surfaceType =
      common::SphericalCoordinates::Convert(
        spherical->Get<std::string>("surface_model"));
    ignition::math::Angle latitude, longitude, heading;
    double elevation = spherical->Get<double>("elevation");
    latitude.Degree(spherical->Get<double>("latitude_deg"));
    longitude.Degree(spherical->Get<double>("longitude_deg"));
    heading.Degree(spherical->Get<double>("heading_deg"));

    this->dataPtr->sphericalCoordinates.reset(new common::SphericalCoordinates(
      surfaceType, latitude, longitude, elevation, heading));
  }


  if (this->dataPtr->sphericalCoordinates == nullptr)
    gzthrow("Unable to create spherical coordinates data structure\n");

  std::string sphericalCoordinatesSurfaceService("/spherical_coordinates_surface_type");
  if (!this->dataPtr->ignNode.Advertise(sphericalCoordinatesSurfaceService,
      &World::SphericalCoordinatesSurfaceService, this))
  {
    gzerr << "Error advertising service [" <<
        sphericalCoordinatesSurfaceService << "]" << std::endl;
  }

  this->dataPtr->rootElement.reset(new Base(BasePtr()));
  this->dataPtr->rootElement->SetName(this->Name());
  this->dataPtr->rootElement->SetWorld(shared_from_this());

  // A special order is necessary when loading a world that contains state
  // information. The joints must be created last, otherwise they get
  // initialized improperly.
  {
    // Create all the entities
    this->LoadEntities(this->dataPtr->sdf, this->dataPtr->rootElement);

    for (unsigned int i = 0; i < this->ModelCount(); ++i)
      this->ModelByIndex(i)->LoadJoints();
  }

  // TODO: Performance test to see if TBB model updating is necessary
  // Choose threaded or unthreaded model updating depending on the number of
  // models in the scene
  // if (this->ModelCount() < 20)
  this->dataPtr->modelUpdateFunc = &World::ModelUpdateSingleLoop;
  // else
  // this->dataPtr->modelUpdateFunc = &World::ModelUpdateTBB;

  event::Events::worldCreated(this->Name());

  this->dataPtr->userCmdManager = UserCmdManagerPtr(
      new UserCmdManager(shared_from_this()));

  // Initialize the world URI.
  this->dataPtr->uri.Clear();
  this->dataPtr->uri.SetScheme("data");
  this->dataPtr->uri.Path().PushFront(this->Name());
  this->dataPtr->uri.Path().PushFront("world");

  this->RegisterIntrospectionItems();

  this->dataPtr->loaded = true;
}

//////////////////////////////////////////////////
const sdf::ElementPtr World::SDF()
{
  this->UpdateStateSDF();
  return this->dataPtr->sdf;
}

const sdf::World *World::GetSDFDom() const
{
  return this->dataPtr->worldSDFDom.get();
}

//////////////////////////////////////////////////
void World::Save(const std::string &_filename)
{
  this->UpdateStateSDF();
  std::string data;
  data = "<?xml version ='1.0'?>\n";
  data += "<sdf version='" + std::string(SDF_VERSION) + "'>\n";
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
  this->Init(nullptr);
}

//////////////////////////////////////////////////
void World::Init(UpdateScenePosesFunc _func)
{
  if (nullptr == this->dataPtr->rootElement)
  {
    gzerr << "Null root element. Not initializing." << std::endl;
    return;
  }

  // Initialize all the entities (i.e. Model)
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); ++i)
    this->dataPtr->rootElement->GetChild(i)->Init();

  // Initialize the physics engine
  this->dataPtr->physicsEngine->Init();

  this->dataPtr->presetManager = PresetManagerPtr(
      new PresetManager(this->dataPtr->physicsEngine, this->dataPtr->sdf));

  this->dataPtr->testRay = boost::dynamic_pointer_cast<RayShape>(
      this->Physics()->CreateShape("ray", CollisionPtr()));

  this->dataPtr->prevStates[0].SetWorld(shared_from_this());
  this->dataPtr->prevStates[1].SetWorld(shared_from_this());

  this->dataPtr->prevStates[0].SetName(this->Name());
  this->dataPtr->prevStates[1].SetName(this->Name());

  this->dataPtr->updateInfo.worldName = this->Name();

  this->dataPtr->iterations = 0;
  this->dataPtr->logPrevIteration = 0;

  util::DiagnosticManager::Instance()->Init(this->Name());

  util::LogRecord::Instance()->Add(this->Name(), "state.log",
      std::bind(&World::OnLog, this, std::placeholders::_1));

  // Check if we have to insert an object population.
  if (this->dataPtr->sdf->HasElement("population"))
  {
    Population population(this->dataPtr->sdf, shared_from_this());
    population.PopulateAll();
  }

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

  this->dataPtr->updateScenePoses = _func;

  this->dataPtr->initialized = true;

  // Mark the world initialization
  gzlog << "Init world[" << this->Name() << "]" << std::endl;
}

//////////////////////////////////////////////////
void World::Run(const unsigned int _iterations)
{
  this->dataPtr->stop = false;
  this->dataPtr->stopIterations = _iterations;

  this->dataPtr->thread = new std::thread(std::bind(&World::RunLoop, this));
}

//////////////////////////////////////////////////
void World::RunBlocking(const unsigned int _iterations)
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
bool World::Running() const
{
  return !this->dataPtr->stop;
}

//////////////////////////////////////////////////
void World::Stop()
{
  this->dataPtr->stop = true;

  // Make sure that the thread does not try to join with itself
  if (this->dataPtr->thread &&
     this->dataPtr->thread->get_id() != std::this_thread::get_id())
  {
    this->dataPtr->thread->join();
    delete this->dataPtr->thread;
    this->dataPtr->thread = nullptr;
  }

  event::Events::stop();
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
    new std::thread(std::bind(&World::LogWorker, this));

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
      std::lock_guard<std::mutex> lock(this->dataPtr->logMutex);
      this->dataPtr->logCondition.notify_all();
    }
    this->dataPtr->logThread->join();
    delete this->dataPtr->logThread;
    this->dataPtr->logThread = nullptr;
  }
}

//////////////////////////////////////////////////
void World::LogStep()
{
  {
    std::lock_guard<std::recursive_mutex> lk(this->dataPtr->worldUpdateMutex);

    if (!this->IsPaused() || this->dataPtr->stepInc != 0)
    {
      if (!this->IsPaused() && this->dataPtr->stepInc == 0)
        this->dataPtr->stepInc = 1;

      std::string data;
      if (!util::LogPlay::Instance()->Step(this->dataPtr->stepInc, data))
      {
        // There are no more chunks, time to exit.
        this->SetPaused(true);
        this->dataPtr->stepInc = 0;
      }
      else
      {
        this->dataPtr->stepInc = 1;

        this->dataPtr->logPlayStateSDF->Clear();
        sdf::readString(data, this->dataPtr->logPlayStateSDF);

        this->dataPtr->logPlayState.Load(this->dataPtr->logPlayStateSDF);

        // If it's the first step, we're going back in time or
        // rt factor is close to zero, don't sleep.
        if ((this->dataPtr->logPlayRealTimeFactor > 1e-5) &&
            (this->dataPtr->logLastStatePlayedRealTime != common::Time(0)) &&
            (this->dataPtr->logLastStatePlayedSimTime != common::Time(0)) &&
            (this->dataPtr->logLastStatePlayedSimTime <
               this->dataPtr->logPlayState.GetSimTime()))
        {
          common::Time timeUntilNextStep =
              common::Time((this->dataPtr->logPlayState.GetSimTime()
                           - this->dataPtr->logLastStatePlayedSimTime).Double()
                           / this->dataPtr->logPlayRealTimeFactor);
          common::Time realTimeOfNextStep =
              this->dataPtr->logLastStatePlayedRealTime + timeUntilNextStep;
          common::Time realTimeSleep =
              realTimeOfNextStep - common::Time::GetWallTime();
          if (realTimeSleep > common::Time(0))
          {
            common::Time::Sleep(realTimeSleep);
          }
        }

        // If the log file does not contain iterations we have to manually
        // increase the iteration counter in logPlayState.
        if (!util::LogPlay::Instance()->HasIterations())
        {
          this->dataPtr->logPlayState.SetIterations(
            this->dataPtr->iterations + 1);
        }

        this->dataPtr->logLastStatePlayedRealTime = common::Time::GetWallTime();
        this->dataPtr->logLastStatePlayedSimTime =
            this->dataPtr->logPlayState.GetSimTime();
        this->SetState(this->dataPtr->logPlayState);
        this->Update();
      }

      if (this->dataPtr->stepInc > 0)
        this->dataPtr->stepInc--;
    }
  }

  this->PublishWorldStats();

  this->ProcessMessages();
}

//////////////////////////////////////////////////
void World::_SetSensorsInitialized(const bool _init)
{
  this->dataPtr->sensorsInitialized = _init;
}

//////////////////////////////////////////////////
bool World::SensorsInitialized() const
{
  return this->dataPtr->sensorsInitialized;
}

/////////////////////////////////////////////////
void World::SetSensorWaitFunc(std::function<void(double, double)> _func)
{
  this->dataPtr->waitForSensors = _func;
}

//////////////////////////////////////////////////
void World::Step()
{
  DIAG_TIMER_START("World::Step");

  IGN_PROFILE("World::Step");

  IGN_PROFILE_BEGIN("lockMutex");
  std::lock_guard<std::mutex> lock(this->dataPtr->stepMutex);
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("loadPlugins");
  /// need this because ODE does not call dxReallocateWorldProcessContext()
  /// until dWorld.*Step
  /// Plugins that manipulate joints (and probably other properties) require
  /// one iteration of the physics engine. Do not remove this.
  if (!this->dataPtr->pluginsLoaded && this->SensorsInitialized())
  {
    this->LoadPlugins();
    this->dataPtr->pluginsLoaded = true;
  }

  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Step", "loadPlugins");

  IGN_PROFILE_BEGIN("publishWorldStats");
  // Send statistics about the world simulation
  this->PublishWorldStats();
  IGN_PROFILE_END();

  DIAG_TIMER_LAP("World::Step", "publishWorldStats");

  IGN_PROFILE_BEGIN("waitForSensors");
  if (this->dataPtr->waitForSensors)
    this->dataPtr->waitForSensors(this->dataPtr->simTime.Double(),
        this->dataPtr->physicsEngine->GetMaxStepSize());
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("sleepOffset");
  double updatePeriod = this->dataPtr->physicsEngine->GetUpdatePeriod();
  // sleep here to get the correct update rate
  common::Time tmpTime = common::Time::GetWallTime();
  common::Time sleepTime = this->dataPtr->prevStepWallTime +
    common::Time(updatePeriod) - tmpTime - this->dataPtr->sleepOffset;

  common::Time actualSleep;
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

  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Step", "sleepOffset");

  IGN_PROFILE_BEGIN("worldUpdateMutex");
  // throttling update rate, with sleepOffset as tolerance
  // the tolerance is needed as the sleep time is not exact
  if (common::Time::GetWallTime() - this->dataPtr->prevStepWallTime +
      this->dataPtr->sleepOffset >= common::Time(updatePeriod))
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->worldUpdateMutex);

    DIAG_TIMER_LAP("World::Step", "worldUpdateMutex");

    this->dataPtr->prevStepWallTime = common::Time::GetWallTime();

    double stepTime = this->dataPtr->physicsEngine->GetMaxStepSize();

    if (!this->IsPaused() || this->dataPtr->stepInc > 0
        || this->dataPtr->needsReset)
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
      if (util::LogRecord::Instance()->BufferSize() > 0)
        util::LogRecord::Instance()->Notify();
      this->dataPtr->pauseTime += stepTime;
    }
  }
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("IntrospectionManager->NotifyUpdates");
  gazebo::util::IntrospectionManager::Instance()->NotifyUpdates();
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("ProcessMessages");
  this->ProcessMessages();
  IGN_PROFILE_END();

  DIAG_TIMER_STOP("World::Step");

  IGN_PROFILE_BEGIN("ClearModels");
  if (g_clearModels)
    this->ClearModels();
  IGN_PROFILE_END();
}

//////////////////////////////////////////////////
void World::Step(const unsigned int _steps)
{
  if (!this->IsPaused())
  {
    gzwarn << "Calling World::Step(steps) while world is not paused\n";
    this->SetPaused(true);
  }

  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->worldUpdateMutex);
    this->dataPtr->stepInc = _steps;
  }

  // block on completion
  bool wait = true;
  while (wait)
  {
    common::Time::NSleep(1);
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->worldUpdateMutex);
    if (this->dataPtr->stepInc == 0 || this->dataPtr->stop)
      wait = false;
  }
}

//////////////////////////////////////////////////
void World::Update()
{
  DIAG_TIMER_START("World::Update");

  IGN_PROFILE("World::Update");
  IGN_PROFILE_BEGIN("needsReset");
  if (this->dataPtr->needsReset)
  {
    if (this->dataPtr->resetAll)
      this->Reset();
    else if (this->dataPtr->resetTimeOnly)
      this->ResetTime();
    else if (this->dataPtr->resetModelOnly)
      this->ResetEntities(Base::MODEL);
    this->dataPtr->needsReset = false;
    return;
  }
  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "needsReset");

  IGN_PROFILE_BEGIN("worldUpdateBegin");
  this->dataPtr->updateInfo.simTime = this->SimTime();
  this->dataPtr->updateInfo.realTime = this->RealTime();
  event::Events::worldUpdateBegin(this->dataPtr->updateInfo);
  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "Events::worldUpdateBegin");

  IGN_PROFILE_BEGIN("Update");
  // Update all the models
  (*this.*dataPtr->modelUpdateFunc)();
  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "Model::Update");

  IGN_PROFILE_BEGIN("UpdateCollision");
  // This must be called before PhysicsEngine::UpdatePhysics for ODE.
  this->dataPtr->physicsEngine->UpdateCollision();
  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "PhysicsEngine::UpdateCollision");

  IGN_PROFILE_BEGIN("beforePhysicsUpdate");
  // Wait for logging to finish, if it's running.
  if (util::LogRecord::Instance()->Running())
  {
    std::unique_lock<std::mutex> lock(this->dataPtr->logMutex);

    // It's possible the logWorker thread never processed the previous
    // state. This checks to make sure that we don't continute until the log
    // worker catchs up.
    if (this->dataPtr->iterations - this->dataPtr->logPrevIteration > 1)
    {
      this->dataPtr->logCondition.notify_one();
      this->dataPtr->logContinueCondition.wait(lock);
    }
  }

  // Give clients a possibility to react to collisions before the physics
  // gets updated.
  this->dataPtr->updateInfo.realTime = this->RealTime();
  event::Events::beforePhysicsUpdate(this->dataPtr->updateInfo);

  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "Events::beforePhysicsUpdate");

  // Update the physics engine
  if (this->dataPtr->enablePhysicsEngine && this->dataPtr->physicsEngine)
  {
    IGN_PROFILE_BEGIN("UpdatePhysics");
    // This must be called directly after PhysicsEngine::UpdateCollision.
    this->dataPtr->physicsEngine->UpdatePhysics();

    IGN_PROFILE_END();
    DIAG_TIMER_LAP("World::Update", "PhysicsEngine::UpdatePhysics");

    // do this after physics update as
    //   ode --> MoveCallback sets the dirtyPoses
    //           and we need to propagate it into Entity::worldPose
    {
      IGN_PROFILE_BEGIN("SetWorldPose(dirtyPoses)");
      // block any other pose updates (e.g. Joint::SetPosition)
      boost::recursive_mutex::scoped_lock plock(
          *this->Physics()->GetPhysicsUpdateMutex());

      for (auto &dirtyEntity : this->dataPtr->dirtyPoses)
      {
        dirtyEntity->SetWorldPose(dirtyEntity->DirtyPose(), false);
      }

      this->dataPtr->dirtyPoses.clear();
      IGN_PROFILE_END();
    }

    DIAG_TIMER_LAP("World::Update", "SetWorldPose(dirtyPoses)");
  }

  IGN_PROFILE_BEGIN("LogRecordNotify");
  // Only update state information if logging data.
  if (util::LogRecord::Instance()->Running())
    this->dataPtr->logCondition.notify_one();
  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "LogRecordNotify");

  IGN_PROFILE_BEGIN("PublishContacts");
  // Output the contact information
  this->dataPtr->physicsEngine->GetContactManager()->PublishContacts();

  IGN_PROFILE_END();
  DIAG_TIMER_LAP("World::Update", "ContactManager::PublishContacts");

  event::Events::worldUpdateEnd();

  gazebo::util::IntrospectionManager::Instance()->Update();

  DIAG_TIMER_STOP("World::Update");
}

//////////////////////////////////////////////////
void World::Fini()
{
  this->dataPtr->stop = true;
  this->dataPtr->enablePhysicsEngine = false;

  // wait until World::Step has completed before proceeding
  std::lock_guard<std::mutex> lock(this->dataPtr->stepMutex);

#ifdef HAVE_OPENAL
  util::OpenAL::Instance()->Fini();
#endif

  // Clean transport
  {
    // Clear subscribers first
    this->dataPtr->controlSub.reset();
    this->dataPtr->factorySub.reset();
    this->dataPtr->jointSub.reset();
    this->dataPtr->lightFactorySub.reset();
    this->dataPtr->lightModifySub.reset();
    this->dataPtr->lightSub.reset();
    this->dataPtr->modelSub.reset();
    this->dataPtr->playbackControlSub.reset();
    this->dataPtr->requestSub.reset();

    this->dataPtr->deleteEntity.clear();
    this->dataPtr->requestMsgs.clear();
    this->dataPtr->factoryMsgs.clear();
    this->dataPtr->modelMsgs.clear();
    this->dataPtr->lightFactoryMsgs.clear();
    this->dataPtr->lightModifyMsgs.clear();
    this->dataPtr->playbackControlMsgs.clear();

    this->dataPtr->poseLocalPub.reset();
    this->dataPtr->posePub.reset();
    this->dataPtr->guiPub.reset();
    this->dataPtr->responsePub.reset();
    this->dataPtr->statPub.reset();
    this->dataPtr->modelPub.reset();
    this->dataPtr->lightPub.reset();
    this->dataPtr->lightFactoryPub.reset();

    if (this->dataPtr->node)
      this->dataPtr->node->Fini();
    this->dataPtr->node.reset();
  }

  this->dataPtr->connections.clear();

  this->dataPtr->sdf.reset();

  this->dataPtr->testRay.reset();
  this->dataPtr->plugins.clear();

  this->dataPtr->publishModelPoses.clear();
  this->dataPtr->publishModelScales.clear();
  this->dataPtr->publishLightPoses.clear();

  // Clean entities
  for (auto &model : this->dataPtr->models)
  {
    if (model)
      model->Fini();
  }
  this->dataPtr->models.clear();

  for (auto &road : this->dataPtr->roads)
  {
    if (road)
      road->Fini();
  }
  this->dataPtr->roads.clear();

  for (auto &light : this->dataPtr->lights)
  {
    if (light)
      light->Fini();
  }
  this->dataPtr->lights.clear();

  if (this->dataPtr->rootElement)
  {
    this->dataPtr->rootElement->Fini();
    this->dataPtr->rootElement.reset();
  }
  this->dataPtr->prevStates[0].SetWorld(WorldPtr());
  this->dataPtr->prevStates[1].SetWorld(WorldPtr());
  this->dataPtr->prevUnfilteredState.SetWorld(WorldPtr());
  this->dataPtr->logPlayState.SetWorld(WorldPtr());
  this->dataPtr->states[0].clear();
  this->dataPtr->states[1].clear();

  this->dataPtr->presetManager.reset();
  this->dataPtr->userCmdManager.reset();

  this->dataPtr->atmosphere.reset();
  this->dataPtr->wind.reset();

  // Engine shouldn't outlive world
  if (this->dataPtr->physicsEngine)
    this->dataPtr->physicsEngine->Fini();
  this->dataPtr->physicsEngine.reset();

  // Clear singletons whose states are tied to this world
  util::DiagnosticManager::Instance()->Fini();
  util::LogRecord::Instance()->Fini();

  // End world run thread
  if (this->dataPtr->thread)
  {
    this->dataPtr->thread->join();
    delete this->dataPtr->thread;
    this->dataPtr->thread = nullptr;
  }

  this->UnregisterIntrospectionItems();
}

//////////////////////////////////////////////////
void World::Clear()
{
  g_clearModels = true;
  /// \todo Clear lights too?
}

//////////////////////////////////////////////////
void World::ClearModels()
{
  g_clearModels = false;
  bool pauseState = this->IsPaused();
  this->SetPaused(true);

  while (!this->dataPtr->models.empty())
  {
    this->RemoveModel(this->dataPtr->models[0]);
  }
  this->dataPtr->models.clear();

  for (auto &road : this->dataPtr->roads)
  {
    if (road)
      road->Fini();
  }
  this->dataPtr->roads.clear();

  this->SetPaused(pauseState);
}

//////////////////////////////////////////////////
std::string World::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
PhysicsEnginePtr World::Physics() const
{
  return this->dataPtr->physicsEngine;
}

//////////////////////////////////////////////////
Wind &World::Wind() const
{
  return *this->dataPtr->wind;
}

//////////////////////////////////////////////////
Atmosphere &World::Atmosphere() const
{
  return *this->dataPtr->atmosphere;
}

//////////////////////////////////////////////////
PresetManagerPtr World::PresetMgr() const
{
  return this->dataPtr->presetManager;
}

//////////////////////////////////////////////////
common::SphericalCoordinatesPtr World::SphericalCoords() const
{
  return this->dataPtr->sphericalCoordinates;
}

//////////////////////////////////////////////////
ignition::math::Vector3d World::Gravity() const
{
  return this->dataPtr->sdf->Get<ignition::math::Vector3d>("gravity");
}

//////////////////////////////////////////////////
void World::SetGravity(const ignition::math::Vector3d &_gravity)
{
  // This function calls `PhysicsEngine::SetGravity`,
  // which in turn should call `World::SetGravitySDF`.
  this->dataPtr->physicsEngine->SetGravity(_gravity);
}

//////////////////////////////////////////////////
void World::SetGravitySDF(const ignition::math::Vector3d &_gravity)
{
  this->dataPtr->sdf->GetElement("gravity")->Set(_gravity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d World::MagneticField() const
{
  return this->dataPtr->sdf->Get<ignition::math::Vector3d>("magnetic_field");
}

//////////////////////////////////////////////////
void World::SetMagneticField(const ignition::math::Vector3d &_mag)
{
  this->dataPtr->sdf->GetElement("magnetic_field")->Set(_mag);
}

//////////////////////////////////////////////////
BasePtr World::BaseByName(const std::string &_name) const
{
  if (this->dataPtr->rootElement)
    return this->dataPtr->rootElement->GetByName(_name);
  else
    return BasePtr();
}

/////////////////////////////////////////////////
ModelPtr World::ModelById(unsigned int _id) const
{
  return boost::dynamic_pointer_cast<Model>(
      this->dataPtr->rootElement->GetByIdRecursive(_id));
}

//////////////////////////////////////////////////
ModelPtr World::ModelByName(const std::string &_name) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->loadModelMutex);
  return boost::dynamic_pointer_cast<Model>(this->BaseByName(_name));
}

//////////////////////////////////////////////////
LightPtr World::LightByName(const std::string &_name) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->loadLightMutex);
  return boost::dynamic_pointer_cast<physics::Light>(this->BaseByName(_name));
}

//////////////////////////////////////////////////
EntityPtr World::EntityByName(const std::string &_name) const
{
  return boost::dynamic_pointer_cast<Entity>(this->BaseByName(_name));
}

//////////////////////////////////////////////////
ModelPtr World::LoadModel(sdf::ElementPtr _sdf , BasePtr _parent)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->loadModelMutex);
  ModelPtr model;

  if (_sdf->GetName() == "model")
  {
    std::string modelName = _sdf->Get<std::string>("name");
    for (auto const &m : this->dataPtr->models)
    {
      if (m->GetName() == modelName)
      {
        gzwarn << "Model with name [" << modelName << "] already exists. "
          << "Not inserting model. This warning can be ignored in certain "
          << "situations such as rewind during log playback.\n";
        return model;
      }
    }

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
LightPtr World::LoadLight(const sdf::ElementPtr &_sdf, const BasePtr &_parent)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->loadLightMutex);

  if (_sdf->GetName() != "light")
  {
    gzerr << "SDF is missing the <light> tag" << std::endl;
    return nullptr;
  }

  // Add to scene message
  msgs::Light *msg = this->dataPtr->sceneMsg.add_light();
  msg->CopyFrom(msgs::LightFromSDF(_sdf));

  // Create new light object
  LightPtr light(new physics::Light(_parent));
  light->SetStatic(true);
  light->ProcessMsg(*msg);
  light->SetWorld(shared_from_this());
  light->Load(_sdf);
  this->dataPtr->lights.push_back(light);

  // msg should contain scoped name (consistent with other entities)
  msg->set_name(light->GetScopedName());

  // publish after adding light to the lights vector
  // we also process light factory msg in World so this avoids creating a
  // duplicate light
  // Note: models uses /model/info topic. We can consider adding a
  // /light/info topic for this, see issue #2288
  this->dataPtr->lightFactoryPub->Publish(*msg);

  return light;
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

  this->EnableAllModels();
  this->PublishModelPose(actor);
  this->dataPtr->models.push_back(actor);

  return actor;
}

//////////////////////////////////////////////////
RoadPtr World::LoadRoad(sdf::ElementPtr _sdf , BasePtr _parent)
{
  RoadPtr road(new Road(_parent));
  road->Load(_sdf);
  this->dataPtr->roads.push_back(road);
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
      this->LoadLight(childElem, _parent);

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
unsigned int World::ModelCount() const
{
  return this->dataPtr->models.size();
}

//////////////////////////////////////////////////
unsigned int World::LightCount() const
{
  return this->dataPtr->lights.size();
}

//////////////////////////////////////////////////
ModelPtr World::ModelByIndex(const unsigned int _index) const
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
Model_V World::Models() const
{
  return this->dataPtr->models;
}

//////////////////////////////////////////////////
Light_V World::Lights() const
{
  return this->dataPtr->lights;
}

//////////////////////////////////////////////////
void World::ResetTime()
{
  this->dataPtr->simTime = common::Time(this->dataPtr->initialSimTime);
  this->dataPtr->pauseTime = common::Time(0);
  this->dataPtr->startTime = common::Time::GetWallTime();
  this->dataPtr->realTimeOffset = common::Time(0);
  this->dataPtr->iterations = 0;

  if (this->IsPaused())
    this->dataPtr->pauseStartTime = this->dataPtr->startTime;

  // Signal a reset has occurred. The SensorManager listens to this event
  // to reset each sensor's last update time.
  event::Events::timeReset();
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
    std::lock_guard<std::recursive_mutex> lk(this->dataPtr->worldUpdateMutex);

    ignition::math::Rand::Seed(ignition::math::Rand::Seed());
    this->dataPtr->physicsEngine->SetSeed(ignition::math::Rand::Seed());

    this->ResetTime();
    this->ResetEntities(Base::BASE);
    for (auto &plugin : this->dataPtr->plugins)
    {
      plugin->Reset();
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
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); ++i)
    this->dataPtr->rootElement->GetChild(i)->Print("");
}

//////////////////////////////////////////////////
gazebo::common::Time World::SimTime() const
{
  return this->dataPtr->simTime;
}

//////////////////////////////////////////////////
void World::SetSimTime(const common::Time &_t)
{
  this->dataPtr->simTime = _t;
}

//////////////////////////////////////////////////
void World::SetInitialSimTime(const common::Time &_t)
{
  this->dataPtr->initialSimTime = _t;
}

//////////////////////////////////////////////////
gazebo::common::Time World::PauseTime() const
{
  return this->dataPtr->pauseTime;
}

//////////////////////////////////////////////////
gazebo::common::Time World::StartTime() const
{
  return this->dataPtr->startTime;
}

//////////////////////////////////////////////////
common::Time World::RealTime() const
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
void World::SetPaused(const bool _p)
{
  if (this->dataPtr->pause == _p)
    return;

  {
    std::lock_guard<std::recursive_mutex> lk(this->dataPtr->worldUpdateMutex);
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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
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
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->worldUpdateMutex);
    this->dataPtr->stepInc = _data->multi_step();
  }

  if (_data->has_seed())
  {
    ignition::math::Rand::Seed(_data->seed());
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
void World::OnPlaybackControl(ConstLogPlaybackControlPtr &_data)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  this->dataPtr->playbackControlMsgs.push_back(*_data);
}

//////////////////////////////////////////////////
void World::ProcessPlaybackControlMsgs()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->worldUpdateMutex);

  for (auto const &msg : this->dataPtr->playbackControlMsgs)
  {
    if (msg.has_pause())
      this->SetPaused(msg.pause());

    if (msg.has_multi_step())
    {
      // stepWorld is a blocking call so set stepInc directly so that
      // world stats will still be published
      this->SetPaused(true);
      this->dataPtr->stepInc += msg.multi_step();
    }

    if (msg.has_seek())
    {
      common::Time targetSimTime = msgs::Convert(msg.seek());
      util::LogPlay::Instance()->Seek(targetSimTime);
      this->dataPtr->stepInc = 1;
    }

    if (msg.has_rewind() && msg.rewind())
    {
      util::LogPlay::Instance()->Rewind();
      this->dataPtr->stepInc = 1;
      if (!util::LogPlay::Instance()->HasIterations())
        this->dataPtr->iterations = 0;
    }

    if (msg.has_forward() && msg.forward())
    {
      util::LogPlay::Instance()->Forward();
      this->dataPtr->stepInc = -1;
      this->SetPaused(true);
      // ToDo: Update iterations if the log doesn't have it.
    }

    if (msg.has_rt_factor())
    {
      this->dataPtr->logPlayRealTimeFactor = msg.rt_factor();
    }
  }

  this->dataPtr->playbackControlMsgs.clear();
}

//////////////////////////////////////////////////
void World::OnRequest(ConstRequestPtr &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  this->dataPtr->requestMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void World::JointLog(ConstJointPtr &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
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
    else if (_entity->HasType(Entity::LIGHT) &&
        _entity->GetParent() == this->dataPtr->rootElement)
    {
      msgs::Light *lightMsg = _scene.add_light();
      boost::static_pointer_cast<physics::Light>(_entity)->FillMsg(*lightMsg);
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
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); ++i)
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
  for (unsigned int i = 0; i < this->dataPtr->rootElement->GetChildCount(); ++i)
  {
    if (this->dataPtr->rootElement->GetChild(i)->HasType(Base::MODEL))
    {
      ModelPtr model = boost::static_pointer_cast<Model>(
          this->dataPtr->rootElement->GetChild(i));
      model->LoadPlugins(this->dataPtr->modelPluginLoadingTimeout);
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
      gzerr << "World[" << this->Name() << "] is attempting to load "
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
  for (auto plugin = this->dataPtr->plugins.begin();
           plugin != this->dataPtr->plugins.end(); ++plugin)
  {
    if ((*plugin)->GetHandle() == _name)
    {
      this->dataPtr->plugins.erase(plugin);
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
  std::lock_guard<std::mutex> lock(this->dataPtr->entityDeleteMutex);

  for (auto &entityName : this->dataPtr->deleteEntity)
  {
    this->RemoveModel(entityName);
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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  msgs::Response response;

  for (auto const &requestMsg : this->dataPtr->requestMsgs)
  {
    bool send = true;
    response.set_id(requestMsg.id());
    response.set_request(requestMsg.request());
    response.set_response("success");

    if (requestMsg.request() == "entity_list")
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
    else if (requestMsg.request() == "entity_delete")
    {
      std::lock_guard<std::mutex> lock2(this->dataPtr->entityDeleteMutex);
      this->dataPtr->deleteEntity.push_back(requestMsg.data());
    }
    else if (requestMsg.request() == "entity_info")
    {
      BasePtr entity(
        this->dataPtr->rootElement->GetByName(requestMsg.data()));
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
        response.set_response("nonexistent");
      }
    }
    else if (requestMsg.request().find("world_sdf") != std::string::npos)
    {
      this->UpdateStateSDF();

      sdf::ElementPtr newSdf(this->dataPtr->sdf);

      // FIXME: Handle scale better on the server so we don't need to unscale
      // SDF here. Issue #1825
      if (requestMsg.request() == "world_sdf_save")
      {
        // Substitute all models sdf with unscaled versions
        if (newSdf->HasElement("model"))
        {
          auto modelElem = newSdf->GetElement("model");
          while (modelElem)
          {
            auto name = modelElem->GetAttribute("name")->GetAsString();
            auto model = this->ModelByName(name);
            if (model)
            {
              auto unscaled = model->UnscaledSDF()->Clone();

              modelElem->Copy(unscaled);
            }

            modelElem = modelElem->GetNextElement("model");
          }
        }
      }

      msgs::GzString msg;
      std::ostringstream stream;
      stream << "<?xml version='1.0'?>\n"
             << "<sdf version='" << SDF_VERSION << "'>\n"
             << newSdf->ToString("")
             << "</sdf>";

      msg.set_data(stream.str());

      std::string *serializedData = response.mutable_serialized_data();
      msg.SerializeToString(serializedData);
      response.set_type(msg.GetTypeName());
    }
    else if (requestMsg.request() == "scene_info")
    {
      this->dataPtr->sceneMsg.clear_model();
      this->dataPtr->sceneMsg.clear_light();
      this->BuildSceneMsg(this->dataPtr->sceneMsg, this->dataPtr->rootElement);

      std::string *serializedData = response.mutable_serialized_data();
      this->dataPtr->sceneMsg.SerializeToString(serializedData);
      response.set_type(this->dataPtr->sceneMsg.GetTypeName());

      for (auto road : this->dataPtr->roads)
      {
        // this causes the roads to publish road msgs.
        road->Init();
      }
    }
    else if (requestMsg.request() == "spherical_coordinates_info")
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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  for (auto const &modelMsg : this->dataPtr->modelMsgs)
  {
    ModelPtr model;
    if (modelMsg.has_id())
      model = this->ModelById(modelMsg.id());
    else
      model = this->ModelByName(modelMsg.name());

    if (!model)
      gzerr << "Unable to find model["
            << modelMsg.name() << "] Id[" << modelMsg.id() << "]\n";
    else
    {
      model->ProcessMsg(modelMsg);

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

      this->dataPtr->modelPub->Publish(modelMsg);
    }
  }

  if (!this->dataPtr->modelMsgs.empty())
  {
    this->EnableAllModels();
    this->dataPtr->modelMsgs.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessLightModifyMsgs()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  for (auto const &lightModifyMsg : this->dataPtr->lightModifyMsgs)
  {
    LightPtr light = this->LightByName(lightModifyMsg.name());

    if (!light)
    {
      gzerr << "Light [" << lightModifyMsg.name() << "] not found."
          << " Use topic ~/factory/light to spawn a new light." << std::endl;
      continue;
    }
    else
    {
      // Update in scene message
      for (int i = 0; i < this->dataPtr->sceneMsg.light_size(); ++i)
      {
        if (this->dataPtr->sceneMsg.light(i).name() == lightModifyMsg.name())
        {
          this->dataPtr->sceneMsg.mutable_light(i)->MergeFrom(lightModifyMsg);
          break;
        }
      }

      // Update light object
      light->ProcessMsg(lightModifyMsg);
    }
  }

  if (!this->dataPtr->lightModifyMsgs.empty())
  {
    this->dataPtr->lightModifyMsgs.clear();
  }
}

//////////////////////////////////////////////////
void World::ProcessLightFactoryMsgs()
{
  // LoadLight also publishes to ~/light/factory so copy light factory msgs to
  // avoid deadlock in OnLightFactory callback when trying to lock receiveMutex
  std::list<msgs::Light> lightFactoryMsgsCopy;
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

    std::copy(this->dataPtr->lightFactoryMsgs.begin(),
        this->dataPtr->lightFactoryMsgs.end(),
        std::back_inserter(lightFactoryMsgsCopy));
    this->dataPtr->lightFactoryMsgs.clear();
  }

  for (auto const &lightFactoryMsg : lightFactoryMsgsCopy)
  {
    LightPtr light = this->LightByName(lightFactoryMsg.name());
    if (light)
    {
      continue;
    }
    else
    {
      // Add to world SDF
      sdf::ElementPtr lightSDF = msgs::LightToSDF(lightFactoryMsg);
      lightSDF->SetParent(this->dataPtr->sdf);
      lightSDF->GetParent()->InsertElement(lightSDF);

      // Create new light object
      this->LoadLight(lightSDF, this->dataPtr->rootElement);
    }
  }
}

//////////////////////////////////////////////////
void World::ProcessFactoryMsgs()
{
  IGN_PROFILE("World::ProcessFactoryMsgs");
  std::list<sdf::ElementPtr> modelsToLoad, lightsToLoad;

  std::list<msgs::Factory> factoryMsgsCopy;
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

    std::copy(this->dataPtr->factoryMsgs.begin(),
      this->dataPtr->factoryMsgs.end(),
      std::back_inserter(factoryMsgsCopy));
    this->dataPtr->factoryMsgs.clear();
  }

  for (auto const &factoryMsg : factoryMsgsCopy)
  {

    this->dataPtr->factorySDF->Clear();

    if (factoryMsg.has_sdf() && !factoryMsg.sdf().empty())
    {
      // SDF Parsing happens here
      if (!sdf::readString(factoryMsg.sdf(), this->dataPtr->factorySDF))
      {
        gzerr << "Unable to read sdf string[" << factoryMsg.sdf() << "]\n";
        continue;
      }
    }
    else if (factoryMsg.has_sdf_filename() &&
            !factoryMsg.sdf_filename().empty())
    {
      std::string filename;
      // If http(s), look at Fuel
      auto uri = ignition::common::URI(factoryMsg.sdf_filename());
      if (uri.Valid() && (uri.Scheme() == "https" || uri.Scheme() == "http"))
      {
        filename = common::FuelModelDatabase::Instance()->ModelFile(
            factoryMsg.sdf_filename());
      }
      // Otherwise, look at database
      else
      {
        filename = common::ModelDatabase::Instance()->GetModelFile(
            factoryMsg.sdf_filename());
      }

      if (!sdf::readFile(filename, this->dataPtr->factorySDF))
      {
        gzerr << "Unable to read sdf file [" << filename << "]\n";
        continue;
      }

      common::convertToFullPaths(this->dataPtr->factorySDF->Root());
    }
    else if (factoryMsg.has_clone_model_name())
    {
      ModelPtr model = this->ModelByName(factoryMsg.clone_model_name());
      if (!model)
      {
        gzerr << "Unable to clone model[" << factoryMsg.clone_model_name()
          << "]. Model not found.\n";
        continue;
      }

      this->dataPtr->factorySDF->Root()->InsertElement(
          model->GetSDF()->Clone());

      std::string newName = model->GetName() + "_clone";
      newName = this->UniqueModelName(newName);

      this->dataPtr->factorySDF->Root()->GetElement("model")->GetAttribute(
          "name")->Set(newName);
    }
    else
    {
      gzerr << "Unable to load sdf from factory message."
        << "No SDF or SDF filename specified.\n";
      continue;
    }

    if (factoryMsg.has_edit_name())
    {
      BasePtr base(
        this->dataPtr->rootElement->GetByName(factoryMsg.edit_name()));
      if (base)
      {
        sdf::ElementPtr elem;
        if (this->dataPtr->factorySDF->Root()->GetName() == "sdf")
          elem = this->dataPtr->factorySDF->Root()->GetFirstElement();
        else
          elem = this->dataPtr->factorySDF->Root();

        base->UpdateParameters(elem);
      }
    }
    else
    {
      bool isActor = false;
      bool isModel = false;
      bool isLight = false;

      sdf::ElementPtr elem = this->dataPtr->factorySDF->Root()->Clone();

      if (!elem)
      {
        gzerr << "Invalid SDF:";
        this->dataPtr->factorySDF->Root()->PrintValues("");
        continue;
      }

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
        this->dataPtr->factorySDF->Root()->PrintValues("");
        continue;
      }

      elem->SetParent(this->dataPtr->sdf);
      elem->GetParent()->InsertElement(elem);
      if (factoryMsg.has_pose())
      {
        elem->GetElement("pose")->Set(msgs::ConvertIgn(factoryMsg.pose()));
      }

      if (isActor)
      {
        ActorPtr actor = this->LoadActor(elem, this->dataPtr->rootElement);
        actor->Init();
        actor->LoadPlugins();
      }
      else if (isModel)
      {
        // Make sure model name is unique
        auto entityName = elem->Get<std::string>("name");
        if (entityName.empty())
        {
          gzerr << "Can't load model with empty name" << std::endl;
          continue;
        }

        // Model with the given name already exists
        if (this->ModelByName(entityName))
        {
          // If allow renaming is disabled
          if (!factoryMsg.allow_renaming())
          {
            gzwarn << "A model named [" << entityName << "] already exists "
                  << "and allow_renaming is false. Model won't be inserted."
                  << std::endl;
            continue;
          }

          entityName = this->UniqueModelName(entityName);
          elem->GetAttribute("name")->Set(entityName);
        }

        modelsToLoad.push_back(elem);
      }
      else if (isLight)
      {
        lightsToLoad.push_back(elem);
      }
    }
  }

  // Load models
  for (auto const &elem : modelsToLoad)
  {
    try
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->factoryDeleteMutex);

      ModelPtr model = this->LoadModel(elem, this->dataPtr->rootElement);
      if (model != nullptr)
      {
        model->Init();
        model->LoadPlugins(this->dataPtr->modelPluginLoadingTimeout);
      }
    }
    catch(...)
    {
      gzerr << "Loading model from factory message failed\n";
    }
  }

  // Load lights
  for (auto const &elem : lightsToLoad)
  {
    try
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->factoryDeleteMutex);

      LightPtr light = this->LoadLight(elem, this->dataPtr->rootElement);
      light->Init();
    }
    catch(...)
    {
      gzerr << "Loading light from factory message failed\n";
    }
  }
}

//////////////////////////////////////////////////
ModelPtr World::ModelBelowPoint(const ignition::math::Vector3d &_pt) const
{
  ModelPtr model;
  EntityPtr entity = this->EntityBelowPoint(_pt);

  if (entity)
    model = entity->GetParentModel();

  return model;
}

//////////////////////////////////////////////////
EntityPtr World::EntityBelowPoint(const ignition::math::Vector3d &_pt) const
{
  std::string entityName;
  double dist;
  ignition::math::Vector3d end;

  end = _pt;
  end.Z() -= 1000;

  this->dataPtr->physicsEngine->InitForThread();
  this->dataPtr->testRay->SetPoints(_pt, end);
  this->dataPtr->testRay->GetIntersection(dist, entityName);
  return this->EntityByName(entityName);
}

//////////////////////////////////////////////////
void World::SetState(const WorldState &_state)
{
  this->SetSimTime(_state.GetSimTime());
  this->dataPtr->logRealTime = _state.GetRealTime();
  this->dataPtr->iterations = _state.GetIterations();

  // Insertions (adapted from ProcessFactoryMsgs)
  auto insertions = _state.Insertions();
  for (auto const &insertion : insertions)
  {
    this->dataPtr->factorySDF->Clear();

    std::stringstream sdfStr;
    sdfStr << "<sdf version='" << SDF_VERSION << "'>"
           << insertion
           << "</sdf>";

    // SDF Parsing happens here
    if (!sdf::readString(sdfStr.str(), this->dataPtr->factorySDF))
    {
      gzerr << "Unable to read sdf string[" << insertion << "]" << std::endl;
      continue;
    }

    // Get entity being inserted
    bool isModel = false;
    bool isLight = false;

    auto elem = this->dataPtr->factorySDF->Root()->Clone();

    if (!elem)
    {
      gzerr << "Invalid SDF:" << std::endl;
      this->dataPtr->factorySDF->Root()->PrintValues("");
      continue;
    }

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
    else
    {
      gzerr << "Unable to find a model or light in:" << std::endl;
      this->dataPtr->factorySDF->Root()->PrintValues("");
      continue;
    }

    elem->SetParent(this->dataPtr->sdf);
    elem->GetParent()->InsertElement(elem);

    if (isModel)
    {
      try
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->factoryDeleteMutex);

        ModelPtr model = this->LoadModel(elem, this->dataPtr->rootElement);
        if (model != nullptr)
        {
          model->Init();
          if (!util::LogPlay::Instance()->IsOpen())
            model->LoadPlugins(this->dataPtr->modelPluginLoadingTimeout);
        }
      }
      catch(...)
      {
        gzerr << "Loading model from world state insertion failed" <<
            std::endl;
      }
    }
    else if (isLight)
    {
      try
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->factoryDeleteMutex);

        LightPtr light = this->LoadLight(elem, this->dataPtr->rootElement);
        light->Init();
      }
      catch(...)
      {
        gzerr << "Loading light from world state insertion failed." <<
            std::endl;
      }
    }
  }

  // Model updates
  const ModelState_M modelStates = _state.GetModelStates();
  for (auto const &modelState : modelStates)
  {
    ModelPtr model = this->ModelByName(modelState.second.GetName());
    if (model)
      model->SetState(modelState.second);
    else
      gzerr << "Unable to find model[" << modelState.second.GetName() << "]\n";
  }

  // Light updates
  const LightState_M lightStates = _state.LightStates();
  for (auto const &lightState : lightStates)
  {
    LightPtr light = this->LightByName(lightState.second.GetName());
    if (light)
      light->SetState(lightState.second);
    else
    {
      gzerr << "Unable to find light[" << lightState.second.GetName() << "]"
            << std::endl;
    }
  }

  // Deletions
  auto deletions = _state.Deletions();
  for (auto const &deletion : deletions)
  {
    // This works for models and lights
    this->RemoveModel(deletion);
  }
}

//////////////////////////////////////////////////
void World::InsertModelFile(const std::string &_sdfFilename)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf_filename(_sdfFilename);
  this->dataPtr->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelSDF(const sdf::SDF &_sdf)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdf.ToString());
  this->dataPtr->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
void World::InsertModelString(const std::string &_sdfString)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  msgs::Factory msg;
  msg.set_sdf(_sdfString);
  this->dataPtr->factoryMsgs.push_back(msg);
}

//////////////////////////////////////////////////
std::string World::StripWorldName(const std::string &_name) const
{
  if (_name.find(this->Name() + "::") == 0)
    return _name.substr(this->Name().size() + 2);
  else
    return _name;
}

//////////////////////////////////////////////////
void World::EnableAllModels()
{
  for (auto &model : this->dataPtr->models)
  {
    model->SetEnabled(true);
  }
}

//////////////////////////////////////////////////
void World::DisableAllModels()
{
  for (auto &model : this->dataPtr->models)
  {
    model->SetEnabled(false);
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
void World::LogModelResources()
{
  if (!util::LogRecord::Instance()->RecordResources())
    return;

  std::set<std::string> modelNames;
  std::set<std::string> fileNames;
  auto addModelResource = [&](const std::string &_uri)
  {
    if (_uri.empty())
      return;

    const std::string modelPrefix = "model://";
    const std::string filePrefix = "file://";
    if (_uri.find(modelPrefix) == 0)
    {
      std::string modelName = _uri.substr(modelPrefix.size(),
        _uri.find("/", modelPrefix.size()) - modelPrefix.size());
      modelNames.insert(modelName);
    }
    else if (_uri.find(filePrefix) == 0 || _uri[0] == '/')
    {
      fileNames.insert(_uri);
    }
  };

  // record model resources if option is enabled.
  for (auto const &model : this->dataPtr->models)
  {
    sdf::ElementPtr modelElem = model->GetSDF();
    if (modelElem->HasElement("link"))
    {
      sdf::ElementPtr linkElem = modelElem->GetElement("link");
      while (linkElem)
      {
        if (linkElem->HasElement("visual"))
        {
          sdf::ElementPtr visualElem = linkElem->GetElement("visual");
          while (visualElem)
          {
            sdf::ElementPtr geomElem = visualElem->GetElement("geometry");
            if (geomElem->HasElement("mesh"))
            {
              const std::string meshUri = geomElem->GetElement("mesh")
                  ->Get<std::string>("uri");
              if (!meshUri.empty())
              {
                addModelResource(meshUri);
              }
            }
            if (visualElem->HasElement("material"))
            {
              sdf::ElementPtr matElem = visualElem->GetElement("material");
              if (matElem->HasElement("script"))
              {
                sdf::ElementPtr scriptElem = matElem->GetElement("script");
                if (scriptElem->HasElement("uri"))
                {
                  std::string matUri = scriptElem->Get<std::string>("uri");
                  if (!matUri.empty())
                  {
                    addModelResource(matUri);
                  }
                }
              }
            }
            visualElem = visualElem->GetNextElement("visual");
          }
        }
        if (linkElem->HasElement("collision"))
        {
          sdf::ElementPtr collisionElem = linkElem->GetElement("collision");
          while (collisionElem)
          {
            sdf::ElementPtr geomElem = collisionElem->GetElement("geometry");
            if (geomElem->HasElement("mesh"))
            {
              const std::string meshUri = geomElem->GetElement("mesh")
                  ->Get<std::string>("uri");
              if (!meshUri.empty())
              {
                addModelResource(meshUri);
              }
            }
            collisionElem = collisionElem->GetNextElement("collision");
          }
        }
        linkElem = linkElem->GetNextElement("link");
      }
    }
  }
  if (!util::LogRecord::Instance()->SaveModels(modelNames) ||
      !util::LogRecord::Instance()->SaveFiles(fileNames))
  {
    gzwarn << "Failed to save model resources during logging\n";
  }
}

//////////////////////////////////////////////////
bool World::OnLog(std::ostringstream &_stream)
{
  int bufferIndex = this->dataPtr->currentStateBuffer;
  // Save the entire state when its the first call to OnLog.
  if (util::LogRecord::Instance()->FirstUpdate())
  {
    this->dataPtr->sdf->Update();
    _stream << "<sdf version ='";
    _stream << SDF_VERSION;
    _stream << "'>\n";
    _stream << this->dataPtr->sdf->ToString("");
    _stream << "</sdf>\n";
  }
  else if (this->dataPtr->states[bufferIndex].size() >= 1)
  {
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->logBufferMutex);
      this->dataPtr->currentStateBuffer ^= 1;
    }
    for (auto const &worldState : this->dataPtr->states[bufferIndex])
    {
      _stream << "<sdf version='" << SDF_VERSION << "'>"
              << worldState
              << "</sdf>";
    }

    this->dataPtr->states[bufferIndex].clear();
  }

  // Logging has stopped. Wait for log worker to finish. Output last bit
  // of data, and reset states.
  if (!util::LogRecord::Instance()->Running())
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->logBufferMutex);

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

  this->LogModelResources();

  return true;
}

//////////////////////////////////////////////////
void World::ProcessMessages()
{
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

    if ((this->dataPtr->posePub && this->dataPtr->posePub->HasConnections()) ||
      // When ready to use the direct API for updating scene poses from server,
      // uncomment the following line:
         this->dataPtr->updateScenePoses ||
        (this->dataPtr->poseLocalPub &&
         this->dataPtr->poseLocalPub->HasConnections()))
    {
      msgs::PosesStamped msg;

      // Time stamp this PosesStamped message
      msgs::Set(msg.mutable_time(), this->SimTime());

      if (!this->dataPtr->publishModelPoses.empty() ||
          !this->dataPtr->publishLightPoses.empty())
      {
        for (auto const &model : this->dataPtr->publishModelPoses)
        {
          std::list<ModelPtr> modelList;
          modelList.push_back(model);
          while (!modelList.empty())
          {
            ModelPtr m = modelList.front();
            modelList.pop_front();
            msgs::Pose *poseMsg = msg.add_pose();

            // Publish the model's relative pose
            poseMsg->set_name(m->GetScopedName());
            poseMsg->set_id(m->GetId());
            msgs::Set(poseMsg, m->RelativePose());

            // Publish each of the model's child links relative poses
            Link_V links = m->GetLinks();
            for (auto const &link : links)
            {
              poseMsg = msg.add_pose();
              poseMsg->set_name(link->GetScopedName());
              poseMsg->set_id(link->GetId());
              msgs::Set(poseMsg, link->RelativePose());
            }

            // add all nested models to the queue
            Model_V models = m->NestedModels();
            for (auto const &n : models)
              modelList.push_back(n);
          }
        }

        for (auto const &light : this->dataPtr->publishLightPoses)
        {
          msgs::Pose *poseMsg = msg.add_pose();

          // Publish the light's pose
          poseMsg->set_name(light->GetScopedName());
          poseMsg->set_id(light->GetId());
          msgs::Set(poseMsg, light->RelativePose());
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

      // When ready to use the direct API for updating scene poses from server,
      // uncomment the following lines:
      // Execute callback to export Pose msg
      if (this->dataPtr->updateScenePoses)
      {
        this->dataPtr->updateScenePoses(this->Name(), msg);
      }
    }

    this->dataPtr->publishModelPoses.clear();
    this->dataPtr->publishLightPoses.clear();
  }

  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

    if (this->dataPtr->modelPub && this->dataPtr->modelPub->HasConnections())
    {
      if (!this->dataPtr->publishModelScales.empty())
      {
        for (auto const &model : this->dataPtr->publishModelScales)
        {
          std::list<ModelPtr> modelList;
          modelList.push_back(model);
          while (!modelList.empty())
          {
            ModelPtr m = modelList.front();
            modelList.pop_front();

            // add all nested models to the queue
            Model_V models = m->NestedModels();
            for (auto const &n : models)
              modelList.push_back(n);

            // Publish the model's scale and visual geometry data at the same
            // time to fix race condition on rendering side when updating
            // visuals
            msgs::Model msg;
            msg.set_name(m->GetScopedName());
            msg.set_id(m->GetId());
            Link_V links = m->GetLinks();
            for (auto l : links)
            {
              msgs::Link *linkMsg = msg.add_link();
              linkMsg->set_id(l->GetId());
              linkMsg->set_name(l->GetScopedName());

              // tmpMsg is unused. The Link::FillMsg call is made in order to
              // keep link's visual msgs up-to-date with latest sdf values.
              // The for loop below does the actual copy of visual geom msg.
              {
                msgs::Link tmpMsg;
                l->FillMsg(tmpMsg);
              }

              auto visualMsgs = l->Visuals();
              for (auto v : visualMsgs)
              {
                if (!v.second.has_geometry())
                  continue;
                msgs::Visual *visualMsg = linkMsg->add_visual();
                visualMsg->set_name(v.second.name());
                visualMsg->set_id(v.second.id());
                visualMsg->set_parent_name(v.second.parent_name());
                visualMsg->set_parent_id(v.second.parent_id());
                auto geomMsg = visualMsg->mutable_geometry();
                geomMsg->CopyFrom(v.second.geometry());
              }
            }

            // set scale
            msgs::Set(msg.mutable_scale(), m->Scale());

            this->dataPtr->modelPub->Publish(msg);
          }
        }
      }
    }
    this->dataPtr->publishModelScales.clear();
  }

  if (common::Time::GetWallTime() - this->dataPtr->prevProcessMsgsTime >
      this->dataPtr->processMsgsPeriod)
  {
    this->ProcessPlaybackControlMsgs();
    this->ProcessEntityMsgs();
    this->ProcessRequestMsgs();
    this->ProcessFactoryMsgs();
    this->ProcessModelMsgs();
    this->ProcessLightFactoryMsgs();
    this->ProcessLightModifyMsgs();
    this->dataPtr->prevProcessMsgsTime = common::Time::GetWallTime();
  }
}

//////////////////////////////////////////////////
void World::PublishWorldStats()
{
  this->dataPtr->worldStatsMsg.Clear();

  msgs::Set(this->dataPtr->worldStatsMsg.mutable_sim_time(),
      this->SimTime());
  msgs::Set(this->dataPtr->worldStatsMsg.mutable_real_time(),
      this->RealTime());
  msgs::Set(this->dataPtr->worldStatsMsg.mutable_pause_time(),
      this->PauseTime());

  this->dataPtr->worldStatsMsg.set_iterations(this->dataPtr->iterations);
  this->dataPtr->worldStatsMsg.set_paused(this->IsPaused());

  if (util::LogPlay::Instance()->IsOpen())
  {
    msgs::LogPlaybackStatistics logStats;
    msgs::Set(logStats.mutable_start_time(),
        util::LogPlay::Instance()->LogStartTime());
    msgs::Set(logStats.mutable_end_time(),
        util::LogPlay::Instance()->LogEndTime());

    this->dataPtr->worldStatsMsg.mutable_log_playback_stats()->CopyFrom(
        logStats);
  }

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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

  // Only add if the model name is not in the list
  this->dataPtr->publishModelPoses.insert(_model);
}

//////////////////////////////////////////////////
void World::PublishModelScale(physics::ModelPtr _model)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

  // Only add if the model name is not in the list
  this->dataPtr->publishModelScales.insert(_model);
}

//////////////////////////////////////////////////
void World::PublishLightPose(const physics::LightPtr _light)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

  // Only add if the light name is not in the list
  this->dataPtr->publishLightPoses.insert(_light);
}

//////////////////////////////////////////////////
void World::LogWorker()
{
  std::unique_lock<std::mutex> lock(this->dataPtr->logMutex);

  WorldPtr self = shared_from_this();
  this->dataPtr->logPrevIteration = this->dataPtr->iterations;

  GZ_ASSERT(self, "Self pointer to World is invalid");

  // Init the prevUnfilteredState
  this->dataPtr->prevUnfilteredState.Load(self);

  while (!this->dataPtr->stop)
  {
    // get unfiltered world state
    WorldState unfilteredState;
    {
      std::lock_guard<std::mutex> dLock(this->dataPtr->entityDeleteMutex);
      unfilteredState.Load(self);
    }

    // compute world state diff and find out about insertions and deletions
    std::vector<std::string> insertions;
    std::vector<std::string> deletions;
    bool insertDelete = false;

    {
      WorldState unfilteredDiffState = unfilteredState -
          this->dataPtr->prevUnfilteredState;
      if (!unfilteredDiffState.IsZero())
      {
        insertions = unfilteredDiffState.Insertions();
        deletions = unfilteredDiffState.Deletions();
        insertDelete = !insertions.empty() || !deletions.empty();
      }
    }
    this->dataPtr->prevUnfilteredState = unfilteredState;

    // Throttle state capture based on log recording frequency.
    auto simTime = this->SimTime();
    if ((simTime - this->dataPtr->logLastStateTime >=
        util::LogRecord::Instance()->Period()) || insertDelete)
    {
      int currState = (this->dataPtr->stateToggle + 1) % 2;

      std::string filterStr = util::LogRecord::Instance()->Filter();
      // compute diff for filtered states
      {
        std::lock_guard<std::mutex> dLock(this->dataPtr->entityDeleteMutex);
        this->dataPtr->prevStates[currState].LoadWithFilter(self, filterStr);
      }
      WorldState diffState = this->dataPtr->prevStates[currState] -
          this->dataPtr->prevStates[this->dataPtr->stateToggle];
      this->dataPtr->logPrevIteration = this->dataPtr->iterations;

      if (!diffState.IsZero() || insertDelete)
      {
        this->dataPtr->stateToggle = currState;
        {
          // Store the entire current state (instead of the diffState). A slow
          // moving link may never be captured if only diff state is recorded.
          std::lock_guard<std::mutex> bLock(this->dataPtr->logBufferMutex);

          this->dataPtr->prevStates[currState].SetInsertions(insertions);
          this->dataPtr->prevStates[currState].SetDeletions(deletions);
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

      this->dataPtr->logLastStateTime = simTime;
    }

    this->dataPtr->logContinueCondition.notify_all();

    // Wait until there is work to be done.
    this->dataPtr->logCondition.wait(lock);
  }

  // Make sure nothing is blocked by this thread.
  this->dataPtr->logContinueCondition.notify_all();
}

/////////////////////////////////////////////////
uint32_t World::Iterations() const
{
  return this->dataPtr->iterations;
}

//////////////////////////////////////////////////
void World::RemoveModel(const std::string &_name)
{
  boost::recursive_mutex::scoped_lock plock(
      *this->Physics()->GetPhysicsUpdateMutex());

  std::lock_guard<std::mutex> flock(this->dataPtr->factoryDeleteMutex);

  // Remove all the dirty poses from the delete entity.
  {
    for (auto entity = this->dataPtr->dirtyPoses.begin();
             entity != this->dataPtr->dirtyPoses.end(); ++entity)
    {
      if ((*entity)->GetName() == _name ||
         ((*entity)->GetParent() && (*entity)->GetParent()->GetName() == _name))
      {
        this->dataPtr->dirtyPoses.erase(entity++);
      }
      else
        ++entity;
    }
  }

  // Remove from SDF
  if (this->dataPtr->sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = this->dataPtr->sdf->GetElement("model");
    while (childElem && childElem->Get<std::string>("name") != _name)
      childElem = childElem->GetNextElement("model");
    if (childElem)
    {
      this->dataPtr->sdf->RemoveChild(childElem);
    }
  }

  if (this->dataPtr->sdf->HasElement("light"))
  {
    sdf::ElementPtr childElem = this->dataPtr->sdf->GetElement("light");
    while (childElem && childElem->Get<std::string>("name") != _name)
      childElem = childElem->GetNextElement("light");
    if (childElem)
    {
      this->dataPtr->sdf->RemoveChild(childElem);
    }
  }

  // remove objects in world
  {
    boost::recursive_mutex::scoped_lock lock(
        *this->Physics()->GetPhysicsUpdateMutex());

    // Remove model object
    for (auto model = this->dataPtr->models.begin();
             model != this->dataPtr->models.end(); ++model)
    {
      if ((*model)->GetName() == _name || (*model)->GetScopedName() == _name)
      {
        this->dataPtr->models.erase(model);
        this->dataPtr->rootElement->RemoveChild(_name);
        break;
      }
    }

    // Remove light object
    for (auto light = this->dataPtr->lights.begin();
        light != this->dataPtr->lights.end(); ++light)
    {
      if ((*light)->GetScopedName() == _name)
      {
        if ((*light)->GetParent())
        {
          // Avoid calling: this->dataPtr->rootElement->RemoveChild(_name);
          // which removes the first child it finds with _name, ignoring
          // entity type (model or light) and scoping of names,
          // e.g. In a world with "point" and "parent::point" entities,
          // removing "point" will remove "parent::child" if it's first in the
          // list
          (*light)->GetParent()->RemoveChild(*light);
        }
        this->dataPtr->lights.erase(light);
        break;
      }
    }

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

  // Cleanup the publishModelPoses list.
  {
    std::lock_guard<std::recursive_mutex> lock2(this->dataPtr->receiveMutex);
    for (auto model = this->dataPtr->publishModelPoses.begin();
             model != this->dataPtr->publishModelPoses.end(); ++model)
    {
      if ((*model)->GetName() == _name || (*model)->GetScopedName() == _name)
      {
        this->dataPtr->publishModelPoses.erase(model);
        break;
      }
    }
  }

  // Cleanup the publishLightPoses list.
  {
    std::lock_guard<std::recursive_mutex> lock2(this->dataPtr->receiveMutex);
    for (auto light : this->dataPtr->publishLightPoses)
    {
      if (light->GetName() == _name || light->GetScopedName() == _name)
      {
        this->dataPtr->publishLightPoses.erase(light);
        break;
      }
    }
  }
}

/////////////////////////////////////////////////
void World::OnLightModifyMsg(ConstLightPtr &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  this->dataPtr->lightModifyMsgs.push_back(*_msg);
}

/////////////////////////////////////////////////
void World::OnLightFactoryMsg(ConstLightPtr &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);
  this->dataPtr->lightFactoryMsgs.push_back(*_msg);
}

/////////////////////////////////////////////////
msgs::Scene World::SceneMsg() const
{
  return this->dataPtr->sceneMsg;
}

/////////////////////////////////////////////////
std::mutex &World::WorldPoseMutex() const
{
  return this->dataPtr->setWorldPoseMutex;
}

/////////////////////////////////////////////////
bool World::PhysicsEnabled() const
{
  return this->dataPtr->enablePhysicsEngine;
}

/////////////////////////////////////////////////
void World::SetPhysicsEnabled(const bool _enable)
{
  this->dataPtr->enablePhysicsEngine = _enable;
}

/////////////////////////////////////////////////
bool World::WindEnabled() const
{
  return this->dataPtr->enableWind;
}

/////////////////////////////////////////////////
void World::SetWindEnabled(const bool _enable)
{
  if (this->dataPtr->enableWind == _enable)
    return;

  this->dataPtr->enableWind = _enable;

  for (auto const &model : this->dataPtr->models)
  {
    Link_V links = model->GetLinks();
    for (auto const &link : links)
    {
      if (link->WindMode())
        link->SetWindEnabled(this->dataPtr->enableWind);
    }
  }
}

/////////////////////////////////////////////////
bool World::AtmosphereEnabled() const
{
  return this->dataPtr->enableAtmosphere;
}

/////////////////////////////////////////////////
void World::SetAtmosphereEnabled(const bool _enable)
{
  this->dataPtr->enableAtmosphere = _enable;
}

/////////////////////////////////////////////////
void World::_AddDirty(Entity *_entity)
{
  GZ_ASSERT(_entity != nullptr, "_entity is nullptr");
  this->dataPtr->dirtyPoses.push_back(_entity);
}

/////////////////////////////////////////////////
void World::ResetPhysicsStates()
{
  for (auto &model : this->dataPtr->models)
    model->ResetPhysicsStates();
}

/////////////////////////////////////////////////
common::URI World::URI() const
{
  return this->dataPtr->uri;
}

/////////////////////////////////////////////////
void World::RegisterIntrospectionItems()
{
  auto uri = this->URI();

  common::URI timeURI(uri);
  timeURI.Query().Insert("p", "time/sim_time");
  this->dataPtr->introspectionItems.push_back(timeURI);
  // Add here all the items that might be introspected.
  gazebo::util::IntrospectionManager::Instance()->Register<common::Time>(
      timeURI.Str(), std::bind(&World::SimTime, this));
}

/////////////////////////////////////////////////
void World::UnregisterIntrospectionItems()
{
  for (auto &item : this->dataPtr->introspectionItems)
    util::IntrospectionManager::Instance()->Unregister(item.Str());

  this->dataPtr->introspectionItems.clear();
}

//////////////////////////////////////////////////
std::string World::UniqueModelName(const std::string &_name)
{
  std::string result = _name;

  int i = 0;
  while (this->ModelByName(result))
    result = _name + "_" + std::to_string(i++);

  return result;
}

//////////////////////////////////////////////////
bool World::PluginInfoService(const ignition::msgs::StringMsg &_req,
                              ignition::msgs::Plugin_V &_plugins)
{
  _plugins.clear_plugins();

  common::URI pluginUri = _req.data();
  if (!pluginUri.Valid())
  {
    gzwarn << "URI [" << _req.data() << "] is not valid." << std::endl;
    return false;
  }

  if (!pluginUri.Path().Contains(this->URI().Path()))
  {
    gzwarn << "Plugin [" << pluginUri.Str() << "] does not match world [" <<
        this->URI().Str() << "]" << std::endl;
    return false;
  }

  auto parts = common::split(pluginUri.Path().Str(), "/");
  auto myParts = common::split(this->URI().Path().Str(), "/");

  for (size_t i = myParts.size(); i < parts.size(); i = i+2)
  {
    // See if there is a model
    if (parts[i] == "model")
    {
      auto model = this->ModelByName(parts[i+1]);

      if (!model)
      {
        gzwarn << "Model [" << parts[i+1] << "] not found in world [" <<
            this->Name() << "]" << std::endl;
        return false;
      }

      bool success = false;
      model->PluginInfo(pluginUri, _plugins, success);
      return success;
    }
    // TODO: Handle world plugins
    else
    {
      gzwarn << "Segment [" << parts[i] << "] in [" << pluginUri.Str() <<
         "] cannot be handled." << std::endl;
      return false;
    }
  }

  gzwarn << "Couldn't get information for plugin [" << pluginUri.Str() << "]"
      << std::endl;

  return false;
}

//////////////////////////////////////////////////
bool World::SceneInfoService(msgs::Scene &_res)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->receiveMutex);

  // Copy implementation from ProcessRequestMsgs
  this->dataPtr->sceneMsg.clear_model();
  this->dataPtr->sceneMsg.clear_light();
  this->BuildSceneMsg(this->dataPtr->sceneMsg, this->dataPtr->rootElement);

  _res = this->dataPtr->sceneMsg;

  for (auto road : this->dataPtr->roads)
  {
    // this causes the roads to publish road msgs.
    road->Init();
  }

  return true;
}

//////////////////////////////////////////////////
bool World::ShadowCasterMaterialNameService(ignition::msgs::StringMsg &_res)
{
  _res.set_data(this->dataPtr->shadowCasterMaterialName.c_str());
  return true;
}

//////////////////////////////////////////////////
bool World::SphericalCoordinatesSurfaceService(ignition::msgs::StringMsg &_res)
{
  if (this->dataPtr->sphericalCoordinates != nullptr)
  {
    if (this->dataPtr->sphericalCoordinates->GetSurfaceType()
        == common::SphericalCoordinates::MOON_SCS)
    {
      _res.set_data("MOON_SCS");
      return true;
    }
    if (this->dataPtr->sphericalCoordinates->GetSurfaceType()
        == common::SphericalCoordinates::EARTH_WGS84)
    {
      _res.set_data("EARTH_WGS84");
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
bool World::ShadowCasterRenderBackFacesService(ignition::msgs::Boolean &_res)
{
  _res.set_data(this->dataPtr->shadowCasterRenderBackFaces);
  return true;
}

//////////////////////////////////////////////////
void World::SetVisualShininess(const std::string &_scopedName,
                               double _shininess)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->materialShininessMutex);
  this->dataPtr->materialShininessMap[_scopedName] = _shininess;
}

//////////////////////////////////////////////////
double World::ShininessByScopedName(const std::string &_scopedName) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->materialShininessMutex);
  if (this->dataPtr->materialShininessMap.find(_scopedName) !=
      this->dataPtr->materialShininessMap.end())
  {
    return this->dataPtr->materialShininessMap.at(_scopedName);
  }
  return 0.0;
}

//////////////////////////////////////////////////
bool World::MaterialShininessService(
    const ignition::msgs::StringMsg &_req, msgs::Any &_res)
{
  _res.set_type(msgs::Any::DOUBLE);
  _res.set_double_value(this->ShininessByScopedName(_req.data()));
  return true;
}
