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
 * Date: 3 Apr 2007
 */

#ifndef __WORLD_HH__
#define __WORLD_HH__

#include <vector>
#include <list>
#include <string>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include "transport/TransportTypes.hh"

#include "msgs/msgs.h"

#include "common/CommonTypes.hh"
#include "common/Event.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/WorldState.hh"
#include "sdf/sdf.h"

namespace boost
{
  class thread;
  class mutex;
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief The World
    /*
     * The world class keps a list of all models, handles loading and saving,
     * object dynamics and collision detection for contact joints
     */
    class World : public boost::enable_shared_from_this<World>
    {
      /// Private constructor
      public: World(const std::string &_name ="");

      /// Private destructor
      public: ~World();

      /// Load the world using SDF parameters
      /// \param _sdf SDF parameters
      public: void Load(sdf::ElementPtr _sdf);

      public: void Save(const std::string &_filename);

      /// \brief Initialize the world
      public: void Init();

      /// \brief Run the world in a thread
      public: void Run();

      /// \brief Stop the world
      public: void Stop();

      /// Finilize the world
      public: void Fini();

      /// \brief Remove all entities from the world
      public: void Clear();

      /// \brief Get the name of the world
      public: std::string GetName() const;

      /// \brief Get the number of parameters
      public: unsigned int GetParamCount() const;

      /// \brief Get a param
      public: common::Param *GetParam(unsigned int index) const;

      /// Return the physics engine
      /// \return Pointer to the physics engine
      public: PhysicsEnginePtr GetPhysicsEngine() const;

      /// \brief Get the number of models
      public: unsigned int GetModelCount() const;

      /// \brief Get a model based on an index
      public: ModelPtr GetModel(unsigned int index);

      /// \brief Reset the simulation to the initial settings
      public: void Reset(bool _resetTime = true);

      /// \brief Get the selected entity
      public: EntityPtr GetSelectedEntity() const;

      /// \brief Print entity tree
      public: void PrintEntityTree();

      /// Get the simulation time
      /// \return The simulation time
      public: common::Time GetSimTime() const;

      /// \brief Set the sim time
      public: void SetSimTime(common::Time t);

      /// Get the pause time
      /// \return The pause time
      public: common::Time GetPauseTime() const;

      /// Get the start time
      /// \return The start time
      public: common::Time GetStartTime() const;

      /// Get the real time (elapsed time)
      /// \return The real time
      public: common::Time GetRealTime() const;

      /// \brief Returns the state of the simulation true if paused
      public: bool IsPaused() const;

      /// \brief Set whether the simulation is paused
      public: void SetPaused(bool p);

      /// \brief Get an element by name
      public: BasePtr GetByName(const std::string &name);

      /// \brief Get a model by id
      public: ModelPtr GetModelById(unsigned int _id);

      /// \brief Get a model by name DEPRECATED
      public: ModelPtr GetModelByName(const std::string &name)GAZEBO_DEPRECATED;

      /// \brief Get a model by name
      public: ModelPtr GetModel(const std::string &_name);

      /// \brief Get a pointer to a entity based on a name
      public: EntityPtr GetEntityByName(
                  const std::string &_name) GAZEBO_DEPRECATED;

      /// \brief Get a pointer to a entity based on a name
      public: EntityPtr GetEntity(const std::string &_name);

      /// \brief Get the nearest model below a point
      public: ModelPtr GetModelBelowPoint(const math::Vector3 &_pt);

      /// \brief Get the nearest entity below a point
      public: EntityPtr GetEntityBelowPoint(const math::Vector3 &_pt);

      /// \brief Get the current world state
      public: WorldState GetState();

      /// \brief Set the current world state
      public: void SetState(const WorldState &_state);

      /// \brief Insert a model from an SDF file
      public: void InsertModel(const std::string &_sdfFilename);

      /// \brief Insert a model using SDF
      public: void InsertModel(const sdf::SDF &_sdf);

      /// \brief Return a version of the name with "<world_name>::" removed
      public: std::string StripWorldName(const std::string &_name) const;

      /// \brief Load a plugin
      public: void LoadPlugin(const std::string &_filename,
                              const std::string &_name,
                              sdf::ElementPtr _sdf);

      /// \brief Remove a running plugin
      public: void RemovePlugin(const std::string &_name);

      /// \brief Enable all links in all the models
      public: void EnableAllModels();

      /// \brief Disable all links in all the models
      public: void DisableAllModels();

      /// \brief Step callback
      public: void StepWorld(int _steps);

      /// \brief Create all entities
      /// \param _sdf SDF element
      /// \param parent Parent of the model to load
      private: void LoadEntities(sdf::ElementPtr _sdf , BasePtr parent);

      /// \brief Load a model
      private: ModelPtr LoadModel(sdf::ElementPtr _sdf, BasePtr parent);

      /// \brief Load an actor
      private: ActorPtr LoadActor(sdf::ElementPtr _sdf, BasePtr parent);

      /// \brief Function to run physics. Used by physicsThread
      private: void RunLoop();
      private: void Step();

      /// \brief For keeping track of time step throttling
      private: common::Time prevStepWallTime;

      /// \brief Update the world
      private: void Update();

      /// \brief Pause callback
      private: void OnPause(bool p);

      /// \brief Step callback
      private: void OnStep();

      private: void OnControl(ConstWorldControlPtr &data);

      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Set the selected entity
      private: void SetSelectedEntityCB(const std::string &name);

      private: void OnEntitiesRequest(ConstRequestPtr &_data);

      /// \brief Construct a scene message from the known world state
      private: void BuildSceneMsg(msgs::Scene &scene, BasePtr entity);

      private: void JointLog(ConstJointPtr &msg);

      private: void OnFactoryMsg(ConstFactoryPtr &_data);
      private: void OnModelMsg(ConstModelPtr &_msg);

      /// \brief TBB version of model updating
      private: void ModelUpdateTBB();

      /// \brief Single loop verison of model updating
      private: void ModelUpdateSingleLoop();

      private: void LoadPlugin(sdf::ElementPtr _sdf);

      private: void FillModelMsg(msgs::Model &_msg, ModelPtr &_model);

      private: void ProcessEntityMsgs();
      private: void ProcessRequestMsgs();
      private: void ProcessFactoryMsgs();
      private: void ProcessModelMsgs();

      private: void UpdateStateSDF();
      private: void UpdateSDFFromState(const WorldState &_state);

      /// Pointer the physics engine
      private: PhysicsEnginePtr physicsEngine;

      private: BasePtr rootElement;

      /// thread in which the world is updated
      private: boost::thread *thread;

      private: bool stop;

      /// List of all the parameters
      protected: common::Param_V parameters;

      /// The entity currently selected by the user
      private: EntityPtr selectedEntity;

      private: std::vector<google::protobuf::Message> messages;

      private: std::string name;

      /// Current simulation time
      private: common::Time simTime, pauseTime, startTime;
      private: bool pause;
      private: int stepInc;

      private: event::Connection_V connections;

      private: transport::NodePtr node;
      private: transport::PublisherPtr selectionPub;
      private: transport::PublisherPtr statPub, responsePub, modelPub;
      private: transport::PublisherPtr guiPub, scenePub;

      private: transport::SubscriberPtr controlSub;
      private: transport::SubscriberPtr factorySub, jointSub;
      private: transport::SubscriberPtr modelSub, requestSub;

      private: msgs::WorldStatistics worldStatsMsg;
      private: msgs::Scene sceneMsg;

      private: void (World::*modelUpdateFunc)();

      private: common::Time statPeriod;
      private: common::Time prevStatTime;
      private: common::Time pauseStartTime;
      private: common::Time realTimeOffset;

      private: boost::mutex *receiveMutex;

      /// TODO: Add an accessor for this, and make it private
      /// lock all pose updates when worldPose is being updated for a model
      public: boost::recursive_mutex *setWorldPoseMutex;
      private: boost::recursive_mutex *worldUpdateMutex;

      private: sdf::ElementPtr sdf;

      private: std::vector<WorldPluginPtr> plugins;
      private: std::list<std::string> deleteEntity;

      public: std::list<Entity*> dirtyPoses;

      private: std::list<msgs::Request> requestMsgs;
      private: std::list<msgs::Factory> factoryMsgs;
      private: std::list<msgs::Model> modelMsgs;

      private: bool needsReset;
      private: bool initialized;

      private: RayShapePtr testRay;
    };

    /// \}
  }
}
#endif


