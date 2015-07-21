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
#ifndef _GAZEBO_WORLD_PRIVATE_HH_
#define _GAZEBO_WORLD_PRIVATE_HH_

#include <atomic>
#include <deque>
#include <vector>
#include <list>
#include <set>
#include <boost/thread.hpp>
#include <sdf/sdf.hh>
#include <string>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"

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

      /// \brief Pointer the spherical coordinates data.
      public: common::SphericalCoordinatesPtr sphericalCoordinates;

      /// \brief The root of all entities in the world.
      public: BasePtr rootElement;

      /// \brief thread in which the world is updated.
      public: boost::thread *thread;

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

      /// \brief True if simulation is paused.
      public: bool pause;

      /// \brief Number of steps in increment by.
      public: int stepInc;

      /// \brief Stores the simulation time target during a 'seek' operation.
      public: common::Time targetSimTime;

      /// \brief When there is a 'seek' command pending during a log file
      /// playback this member variable should be true.
      public: bool seekPending;

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

      /// \brief Publisher for light messages.
      public: transport::PublisherPtr lightPub;

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
      public: boost::recursive_mutex *receiveMutex;

      /// \brief Mutex to protext loading of models.
      public: boost::mutex *loadModelMutex;

      /// \TODO: Add an accessor for this, and make it private
      /// Used in Entity.cc.
      /// Entity::Reset to call Entity::SetWorldPose and Entity::SetRelativePose
      /// Entity::SetWorldPose to call Entity::setWorldPoseFunc
      public: boost::mutex *setWorldPoseMutex;

      /// \brief Used by World classs in following calls:
      /// World::Step for then entire function
      /// World::StepWorld for changing World::stepInc,
      /// and waits on setpInc on World::stepIhc as it's decremented.
      /// World::Reset while World::ResetTime, entities, World::physicsEngine
      /// World::SetPaused to assign world::pause
      public: boost::recursive_mutex *worldUpdateMutex;

      /// \brief THe world's SDF values.
      public: sdf::ElementPtr sdf;

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

      /// \brief Info passed through the WorldUpdateBegin event.
      public: common::UpdateInfo updateInfo;

      /// \brief The number of simulation iterations.
      public: uint64_t iterations;

      /// \brief The number of simulation iterations to take before stopping.
      public: uint64_t stopIterations;

      /// \brief Condition used for log worker.
      public: boost::condition_variable logCondition;

      /// \brief Condition used to guarantee the log worker thread doesn't
      /// skip an interation.
      public: boost::condition_variable logContinueCondition;

      /// \brief Last iteration recorded by the log worker thread.
      public: uint64_t logPrevIteration;

      /// \brief Real time value set from a log file.
      public: common::Time logRealTime;

      /// \brief Mutex to protect the log worker thread.
      public: boost::mutex logMutex;

      /// \brief Mutex to protect the log state buffers
      public: boost::mutex logBufferMutex;

      /// \brief Mutex to protect the deleteEntity list.
      public: boost::mutex entityDeleteMutex;

      /// \brief Worker thread for logging.
      public: boost::thread *logThread;

      /// \brief A cached list of models. This is here for performance.
      public: Model_V models;

      /// \brief This mutex is used to by the ::RemoveModel and
      /// ::ProcessFactoryMsgs functions.
      public: boost::mutex factoryDeleteMutex;

      /// \brief when physics engine makes an update and changes a link pose,
      /// this flag is set to trigger Entity::SetWorldPose on the
      /// physics::Link in World::Update.
      public: std::list<Entity*> dirtyPoses;

      /// \brief Class to manage preset simulation parameter profiles.
      public: PresetManagerPtr presetManager;

      /// \brief True if sensors have been initialized. This should be set
      /// by the SensorManager.
      public: std::atomic_bool sensorsInitialized;
    };
  }
}

#endif
