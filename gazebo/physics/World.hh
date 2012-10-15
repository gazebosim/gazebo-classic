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
 * Date: 3 Apr 2007
 */

#ifndef _WORLD_HH_
#define _WORLD_HH_

#include <vector>
#include <list>
#include <string>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include "transport/TransportTypes.hh"

#include "msgs/msgs.hh"

#include "common/CommonTypes.hh"
#include "common/Event.hh"

#include "physics/Base.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/WorldState.hh"
#include "sdf/sdf.hh"

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

    /// \class World World.hh physics/World.hh
    /// \brief The world provides access to all other object within a simulated
    /// environment.
    ///
    /// The World is the container for all models and their components
    /// (links, joints, sensors, plugins, etc), and WorldPlugin instances.
    /// Many core function are also handled in the World, including physics
    /// update, model updates, and message processing.
    class World : public boost::enable_shared_from_this<World>
    {
      /// \brief Constructor
      ///
      /// Constructor for the World. Must specify a unique name.
      /// \param _name Name of the world
      public: World(const std::string &_name ="");

      /// \brief Destructor
      public: ~World();

      /// \brief Load the world using SDF parameters
      ///
      /// Load a world from and SDF pointer
      /// \param _sdf SDF parameters
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Save a world to a file
      ///
      /// Save the current world and its state to a file
      /// \param _filename Name of the file to save into
      public: void Save(const std::string &_filename);

      /// \brief Initialize the world
      ///
      /// This is called after Load.
      public: void Init();

      /// \brief Run the world in a thread
      ///
      /// Run the update loop.
      public: void Run();

      /// \brief Stop the world
      ///
      /// Stop the update loop.
      public: void Stop();

      /// \brief Finilize the world
      ///
      /// Call this function to tear-down the world.
      public: void Fini();

      /// \brief Remove all entities from the world
      public: void Clear();

      /// \brief Get the name of the world
      public: std::string GetName() const;

      /// \brief Return the physics engine
      ///
      /// Get a pointer to the physics engine used by the world.
      /// \return Pointer to the physics engine
      public: PhysicsEnginePtr GetPhysicsEngine() const;

      /// \brief Get the number of models
      /// \return The number of models in the World.
      public: unsigned int GetModelCount() const;

      /// \brief Get a model based on an index
      ///
      /// Get a Model using an index, where index must be greater than zero
      /// and less than World::GetModelCount()
      /// \param _index The index of the model [0..GetModelCount)
      /// \return A pointer to the Model. NULL if _index is invalid.
      public: ModelPtr GetModel(unsigned int _index) const;

      /// \brief Get a list of all the models
      /// \return A list of all the Models in the world
      public: std::list<ModelPtr> GetModels() const;

      /// \brief Reset with options
      ///
      /// The _type parameter specifies which type of eneities to reset. See
      /// Base::EntityType.
      /// \param _type The type of reset.
      public: void ResetEntities(Base::EntityType _type = Base::BASE);

      /// \brief Reset simulation time back to zero
      public: void ResetTime();

      /// \brief Reset time and model poses, configurations in simulation
      public: void Reset();

      /// \brief Get the selected Entity
      ///
      /// The selected entity is set via the GUI.
      /// \return A point to the Entity, NULL if nothing is selected
      public: EntityPtr GetSelectedEntity() const;

      /// \brief Print Entity tree
      ///
      /// Prints alls the entities to stdout
      public: void PrintEntityTree();

      /// \brief Get the world simulation time, note if you want the PC
      /// wall clock call World::GetRealTime.
      /// \return The current simulation time
      public: common::Time GetSimTime() const;

      /// \brief Set the sim time
      /// \param _t The new simulation time
      public: void SetSimTime(common::Time _t);

      /// \brief Get the amount of time simulation has been paused
      /// \return The pause time
      public: common::Time GetPauseTime() const;

      /// \brief Get the wall time simulation was started.
      /// \return The start time
      public: common::Time GetStartTime() const;

      /// \brief Get the real time (elapsed time)
      /// \return The real time
      public: common::Time GetRealTime() const;

      /// \brief Returns the state of the simulation true if paused
      /// \return True if paused.
      public: bool IsPaused() const;

      /// \brief Set whether the simulation is paused
      /// \param _p True pauses the simulation. False runs the simulation.
      public: void SetPaused(bool _p);

      /// \brief Get an element by name
      ///
      /// Searches the list of entities, and return a pointer to the model
      /// with a matching _name.
      /// \param _name The name of the Model to find.
      /// \return A pointer to the entity, or NULL if no entity was found.
      public: BasePtr GetByName(const std::string &_name);

      /// \brief Get a model by name DEPRECATED
      public: ModelPtr GetModelByName(const std::string &name)GAZEBO_DEPRECATED;

      /// \brief Get a model by name
      ///
      /// This function is the same as GetByName, but limits the search to
      /// only models.
      /// \param _name The name of the Model to find.
      /// \return A pointer to the Model, or NULL if no model was found.
      public: ModelPtr GetModel(const std::string &_name);

      /// \brief Get a pointer to a entity based on a name
      public: EntityPtr GetEntityByName(
                  const std::string &_name) GAZEBO_DEPRECATED;

      /// \brief Get a pointer to an Entity based on a name
      ///
      /// This function is the same as GetByName, but limits the search to
      /// only Entities.
      /// \param _name The name of the Entity to find.
      /// \return A pointer to the Entity, or NULL if no Entity was found.
      public: EntityPtr GetEntity(const std::string &_name);

      /// \brief Get the nearest model below a point
      ///
      /// This function makes use of World::GetEntityBelowPoint.
      /// \param _pt The 3D point to search below
      /// \return A pointer to nearest Model, NULL if none is found.
      public: ModelPtr GetModelBelowPoint(const math::Vector3 &_pt);

      /// \brief Get the nearest entity below a point
      ///
      /// Projects a Ray down (-Z axis) starting at the given point. The
      /// first entity hit by the Ray is returned.
      /// \param _pt The 3D point to search below
      /// \return A pointer to nearest Entity, NULL if none is found.
      public: EntityPtr GetEntityBelowPoint(const math::Vector3 &_pt);

      /// \brief Get the current world state
      /// \return A object that contains the entire state of the World
      public: WorldState GetState();

      /// \brief Set the current world state
      /// \param _state The state to set the World to.
      public: void SetState(const WorldState &_state);

      /// \brief Insert a model from an SDF file
      ///
      /// Spawns a model into the world base on and SDF file
      /// \param _sdfFilename The name of the SDF file (including path).
      public: void InsertModelFile(const std::string &_sdfFilename);

      /// \brief Insert a model from an SDF string
      ///
      /// Spawns a model into the world base on and SDF string
      /// \param _sdfString A string containing valid SDF markup.
      public: void InsertModelString(const std::string &_sdfString);

      /// \brief Insert a model using SDF
      ///
      /// Spawns a model into the world base on and SDF object
      /// \param _sdf A reference to an SDF object
      public: void InsertModelSDF(const sdf::SDF &_sdf);

      /// \brief Return a version of the name with "<world_name>::" removed
      /// \param _name Usually the name of an entity.
      /// \return The stripped world name
      public: std::string StripWorldName(const std::string &_name) const;

      /// \brief Enable all links in all the models
      ///
      /// Enable is a physics concept. Enabling means that the physics
      /// engine should update an entity.
      public: void EnableAllModels();

      /// \brief Disable all links in all the models
      ///
      /// Disable is a physics concept. Disabling means that the physics
      /// engine should not update an entity.
      public: void DisableAllModels();

      /// \brief Step callback
      /// \param _steps The number of steps the World should take
      public: void StepWorld(int _steps);

      /// \brief Load a plugin
      /// \param _filename The filename of the plugin
      /// \param _name A unique name for the plugin
      /// \param _sdf The SDF to pass into the plugin.
      public: void LoadPlugin(const std::string &_filename,
                               const std::string &_name,
                               sdf::ElementPtr _sdf);

      /// \brief Remove a running plugin
      /// \param _name The unique name of the plugin to remove
      public: void RemovePlugin(const std::string &_name);

      /// \brief Get a model by id
      ///
      /// Each Entity has a unique ID, this function finds a Model with
      /// a passed in _id.
      /// \param _id The id of the Model
      /// \return A pointer to the model, or NULL if no Model was found.
      private: ModelPtr GetModelById(unsigned int _id);

      /// \brief Load all plugins
      ///
      /// Load all plugins specified in the SDF for the model.
      private: void LoadPlugins();

      /// \brief Create and load all entities
      /// \param _sdf SDF element
      /// \param _parent Parent of the model to load
      private: void LoadEntities(sdf::ElementPtr _sdf , BasePtr _parent);

      /// \brief Load a model
      /// \param _sdf SDF element containing the Model description
      /// \param _parent Parent of the model
      /// \return Pointer to the newly created Model
      private: ModelPtr LoadModel(sdf::ElementPtr _sdf, BasePtr _parent);

      /// \brief Load an actor
      /// \param _sdf SDF element containing the Actor description
      /// \param _parent Parent of the Actor
      /// \return Pointer to the newly created Actor
      private: ActorPtr LoadActor(sdf::ElementPtr _sdf, BasePtr _parent);

      /// \brief Load a road
      /// \param _sdf SDF element containing the Road description
      /// \param _parent Parent of the Road
      /// \return Pointer to the newly created Road
      private: RoadPtr LoadRoad(sdf::ElementPtr _sdf , BasePtr _parent);

      /// \brief Function to run physics. Used by physicsThread
      private: void RunLoop();

      /// \brief Step the world once.
      private: void Step();

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

      /// \brief For keeping track of time step throttling
      private: common::Time prevStepWallTime;

      /// Pointer the physics engine
      private: PhysicsEnginePtr physicsEngine;

      private: BasePtr rootElement;

      /// thread in which the world is updated
      private: boost::thread *thread;

      private: bool stop;

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
      private: transport::PublisherPtr guiPub;
      private: transport::PublisherPtr lightPub;

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
      private: boost::mutex *loadModelMutex;

      /// TODO: Add an accessor for this, and make it private
      /// Used in Entity.cc.
      ///   Entity::Reset to call Entity::SetWorldPose and
      ///     Entity::SetRelativePose
      ///   Entity::SetWorldPose to call Entity::setWorldPoseFunc
      private: boost::mutex *setWorldPoseMutex;

      public: boost::mutex *GetSetWorldPoseMutex() const
        { return this->setWorldPoseMutex; };

      /// Used by World classs in following calls:
      ///   World::Step for then entire function
      ///   World::StepWorld for changing World::stepInc,
      ///     and waits on setpInc on World::stepIhc as it's decremented.
      ///   World::Reset while World::ResetTime, entities, World::physicsEngine
      ///   World::SetPuased to assign world::pause
      private: boost::recursive_mutex *worldUpdateMutex;

      private: sdf::ElementPtr sdf;

      private: std::vector<WorldPluginPtr> plugins;
      private: std::list<std::string> deleteEntity;

      /// \brief when physics engine makes an update and changes a link pose,
      ///        this flag is set to trigger Entity::SetWorldPose on the
      ///        physics::Link in World::Update.
      public: std::list<Entity*> dirtyPoses;

      private: std::list<msgs::Request> requestMsgs;
      private: std::list<msgs::Factory> factoryMsgs;
      private: std::list<msgs::Model> modelMsgs;

      private: bool needsReset;
      // Options to reset All, Time Only, or Model Only
      // reset All overrides Time Only and Model Only
      private: bool resetAll;
      private: bool resetTimeOnly;
      private: bool resetModelOnly;

      private: bool initialized;
      private: bool enablePhysicsEngine;

      /// \brief check if physics engine is enabled/disabled
      public: bool GetEnablePhysicsEngine()
              { return this->enablePhysicsEngine; }

      /// \brief enable/disable physics engine during World::Update
      public: void EnablePhysicsEngine(bool _enable)
              { this->enablePhysicsEngine = _enable; }

      private: RayShapePtr testRay;
      private: bool pluginsLoaded;
    };

    /// \}
  }
}
#endif


