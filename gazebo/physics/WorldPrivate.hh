/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_WORLDPRIVATE_HH_
#define GAZEBO_PHYSICS_WORLDPRIVATE_HH_

#include <atomic>
#include <deque>
#include <vector>
#include <list>
#include <memory>
#include <set>
#include <sdf/sdf.hh>
#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <ignition/transport.hh>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/WorldState.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Private data class for World.
    class WorldPrivate
    {
      /// \brief For keeping track of time step throttling.
      public: common::Time prevStepWallTime;

      /// \brief Pointer the physics engine.
      public: PhysicsEnginePtr physicsEngine;

      /// \brief Unique pointer the wind. The world owns this pointer.
      public: std::unique_ptr<Wind> wind;

      /// \brief Unique pointer the atmosphere model.
      /// The world owns this pointer.
      public: std::unique_ptr<Atmosphere> atmosphere;

      /// \brief Pointer the spherical coordinates data.
      public: common::SphericalCoordinatesPtr sphericalCoordinates;

      /// \brief The root of all entities in the world.
      public: BasePtr rootElement;

      /// \brief thread in which the world is updated.
      public: std::thread *thread;

      /// \brief True to stop the world from running.
      public: bool stop;

      /// \brief Name of the world.
      public: std::string name;

      /// \brief Current simulation time.
      public: common::Time simTime;

      /// \brief Amount of time simulation has been paused.
      public: common::Time pauseTime;

      /// \brief Clock time when simulation was started.
      public: common::Time startTime;

      /// \brief Initial simulation time.
      public: common::Time initialSimTime;

      /// \brief True if simulation is paused.
      public: bool pause;

      /// \brief Number of steps in increment by.
      public: int stepInc;

      /// \brief All the event connections.
      public: event::Connection_V connections;

      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Publisher for world statistics messages.
      public: transport::PublisherPtr statPub;

      /// \brief Publisher for request response messages.
      public: transport::PublisherPtr responsePub;

      /// \brief Publisher for model messages.
      public: transport::PublisherPtr modelPub;

      /// \brief Publisher for gui messages.
      public: transport::PublisherPtr guiPub;

      /// \brief Publisher for light modify messages.
      public: transport::PublisherPtr lightPub;

      /// \brief Publisher for light factory messages.
      public: transport::PublisherPtr lightFactoryPub;

      /// \brief Publisher for pose messages.
      public: transport::PublisherPtr posePub;

      /// \brief Publisher for local pose messages.
      public: transport::PublisherPtr poseLocalPub;

      /// \brief Subscriber to world control messages.
      public: transport::SubscriberPtr controlSub;

      /// \brief Subscriber to log playback control messages.
      public: transport::SubscriberPtr playbackControlSub;

      /// \brief Subscriber to factory messages.
      public: transport::SubscriberPtr factorySub;

      /// \brief Subscriber to joint messages.
      public: transport::SubscriberPtr jointSub;

      /// \brief Subscriber to light messages.
      public: transport::SubscriberPtr lightSub;

      /// \brief Subscriber to light factory messages.
      public: transport::SubscriberPtr lightFactorySub;

      /// \brief Subscriber to light modify messages.
      public: transport::SubscriberPtr lightModifySub;

      /// \brief Subscriber to model messages.
      public: transport::SubscriberPtr modelSub;

      /// \brief Subscriber to request messages.
      public: transport::SubscriberPtr requestSub;

      /// \brief Outgoing world statistics message.
      public: msgs::WorldStatistics worldStatsMsg;

      /// \brief Outgoing scene message.
      public: msgs::Scene sceneMsg;

      /// \brief Function pointer to the model update function.
      public: void (World::*modelUpdateFunc)();

      /// \brief Last time a world statistics message was sent.
      public: common::Time prevStatTime;

      /// \brief Time at which pause started.
      public: common::Time pauseStartTime;

      /// \brief Used to compute a more accurate real time value.
      public: common::Time realTimeOffset;

      /// \brief Mutex to protect incoming message buffers.
      public: std::recursive_mutex receiveMutex;

      /// \brief Mutex to protext loading of models.
      public: std::mutex loadModelMutex;

      /// \brief Mutex to protext loading of lights.
      public: std::mutex loadLightMutex;

      /// \TODO: Add an accessor for this, and make it private
      /// Used in Entity.cc.
      /// Entity::Reset to call Entity::SetWorldPose and Entity::SetRelativePose
      /// Entity::SetWorldPose to call Entity::setWorldPoseFunc
      public: std::mutex setWorldPoseMutex;

      /// \brief Used in World::Step and World::Fini
      public: std::mutex stepMutex;

      /// \brief Used by World classs in following calls:
      /// World::Step for then entire function
      /// World::StepWorld for changing World::stepInc,
      /// and waits on setpInc on World::stepIhc as it's decremented.
      /// World::Reset while World::ResetTime, entities, World::physicsEngine
      /// World::SetPaused to assign world::pause
      public: std::recursive_mutex worldUpdateMutex;

      /// \brief The world's current SDF description.
      public: sdf::ElementPtr sdf;

      /// \brief Timeout for Model::LoadPlugins in seconds.
      public: unsigned int modelPluginLoadingTimeout = 30;

      /// \brief All the plugins.
      public: std::vector<WorldPluginPtr> plugins;

      /// \brief List of entities to delete.
      public: std::list<std::string> deleteEntity;

      /// \brief Request message buffer.
      public: std::list<msgs::Request> requestMsgs;

      /// \brief Factory message buffer.
      public: std::list<msgs::Factory> factoryMsgs;

      /// \brief Model message buffer.
      public: std::list<msgs::Model> modelMsgs;

      /// \brief Light factory message buffer.
      public: std::list<msgs::Light> lightFactoryMsgs;

      /// \brief Light modify message buffer.
      public: std::list<msgs::Light> lightModifyMsgs;

      /// \brief Playback control message buffer.
      public: std::list<msgs::LogPlaybackControl> playbackControlMsgs;

      /// \brief True to reset the world on next update.
      public: bool needsReset;

      /// \brief True to reset everything.
      public: bool resetAll;

      /// \brief True to reset only the time.
      public: bool resetTimeOnly;

      /// \brief True to reset only model poses.
      public: bool resetModelOnly;

      /// \brief True if the world has been initialized.
      public: bool initialized;

      /// \brief True if the world has been loaded.
      public: bool loaded;

      /// \brief True to enable the physics engine.
      public: bool enablePhysicsEngine;

      /// \brief True to enable the wind.
      public: bool enableWind;

      /// \brief True to enable the atmosphere model.
      public: bool enableAtmosphere;

      /// \brief Ray used to test for collisions when placing entities.
      public: RayShapePtr testRay;

      /// \brief True if the plugins have been loaded.
      public: bool pluginsLoaded;

      /// \brief sleep timing error offset due to clock wake up latency
      public: common::Time sleepOffset;

      /// \brief Last time incoming messages were processed.
      public: common::Time prevProcessMsgsTime;

      /// \brief Period over which messages should be processed.
      public: common::Time processMsgsPeriod;

      /// \brief Alternating buffer of states.
      public: std::deque<WorldState> states[2];

      /// \brief Keep track of current state buffer being updated
      public: int currentStateBuffer;

      /// \brief Buffer of prev states
      public: WorldState prevStates[2];

      /// \brief Previous unfiltered state. Used for determining insertions
      /// and deletions
      public: WorldState prevUnfilteredState;

      /// \brief Int used to toggle between prevStates
      public: int stateToggle;

      /// \brief State from from log file.
      public: sdf::ElementPtr logPlayStateSDF;

      /// \brief Current state when playing from a log file.
      public: WorldState logPlayState;

      /// \brief Store a factory SDF object to improve speed at which
      /// objects are inserted via the factory.
      public: sdf::SDFPtr factorySDF;

      /// \brief The list of models that need to publish their pose.
      public: std::set<ModelPtr> publishModelPoses;

      /// \brief The list of models that need to publish their scale.
      public: std::set<ModelPtr> publishModelScales;

      /// \brief The list of lights that need to publish their pose.
      public: std::set<LightPtr> publishLightPoses;

      /// \brief Info passed through the WorldUpdateBegin event.
      public: common::UpdateInfo updateInfo;

      /// \brief The number of simulation iterations.
      public: uint64_t iterations;

      /// \brief The number of simulation iterations to take before stopping.
      public: uint64_t stopIterations;

      /// \brief Condition used for log worker.
      public: std::condition_variable logCondition;

      /// \brief Condition used to guarantee the log worker thread doesn't
      /// skip an interation.
      public: std::condition_variable logContinueCondition;

      /// \brief Last iteration recorded by the log worker thread.
      public: uint64_t logPrevIteration;

      /// \brief Real time value set from a log file.
      public: common::Time logRealTime;

      /// \brief Mutex to protect the log worker thread.
      public: std::mutex logMutex;

      /// \brief Mutex to protect the log state buffers
      public: std::mutex logBufferMutex;

      /// \brief Mutex to protect the deleteEntity list.
      public: std::mutex entityDeleteMutex;

      /// \brief Worker thread for logging.
      public: std::thread *logThread;

      /// \brief A cached list of models. This is here for performance.
      public: Model_V models;

      /// \brief A cached list of lights.
      public: Light_V lights;

      /// \brief This mutex is used to by the ::RemoveModel and
      /// ::ProcessFactoryMsgs functions.
      public: std::mutex factoryDeleteMutex;

      /// \brief when physics engine makes an update and changes a link pose,
      /// this flag is set to trigger Entity::SetWorldPose on the
      /// physics::Link in World::Update.
      public: std::list<Entity*> dirtyPoses;

      /// \brief Class to manage preset simulation parameter profiles.
      public: PresetManagerPtr presetManager;

      /// \brief Class to manage user commands.
      public: UserCmdManagerPtr userCmdManager;

      /// \brief True if sensors have been initialized. This should be set
      /// by the SensorManager.
      public: std::atomic_bool sensorsInitialized;

      /// \brief Simulation time of the last log state captured.
      public: gazebo::common::Time logLastStateTime;

      /// \brief Simulation time of the last log state played.
      public: gazebo::common::Time logLastStatePlayedSimTime;

      /// \brief Wall clock time when the last state was played.
      public: gazebo::common::Time logLastStatePlayedRealTime;

      /// \brief Log play real time factor
      public: double logPlayRealTimeFactor;

      /// \brief URI of this world.
      public: common::URI uri;

      /// \brief All the introspection items regsitered for this.
      public: std::vector<common::URI> introspectionItems;

      /// \brief A list of roads in the world
      public: std::vector<RoadPtr> roads;

      /// \brief Node for ignition transport communication.
      public: ignition::transport::Node ignNode;

      /// \brief Wait until no sensors use the current step any more
      public: std::function<void(double, double)> waitForSensors;

      /// \brief Callback function intended to call the scene with updated Poses
      public: UpdateScenePosesFunc updateScenePoses;

      /// \brief SDF World DOM object
      public: std::unique_ptr<sdf::World> worldSDFDom;

      /// \brief Shadow caster material name from scene SDF
      public: std::string shadowCasterMaterialName = "Gazebo/shadow_caster";

      /// \brief Shadow caster render back faces from scene SDF
      public: bool shadowCasterRenderBackFaces = true;

      /// \brief This mutex is used to by the SetVisualShininess and
      /// ShininessByScopedName methods to protect materialShininessMap.
      public: std::mutex materialShininessMutex;

      /// \brief Shininess values from scene SDF
      public: std::map<std::string, double> materialShininessMap;
    };
  }
}

#endif
