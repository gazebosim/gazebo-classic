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
#ifndef GAZEBO_PHYSICS_WORLD_HH_
#define GAZEBO_PHYSICS_WORLD_HH_

#include <vector>
#include <list>
#include <set>
#include <deque>
#include <string>
#include <memory>

#include <boost/enable_shared_from_this.hpp>

#include <sdf/sdf.hh>

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/physics/Base.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/WorldState.hh"
#include "gazebo/physics/Wind.hh"
#include "gazebo/util/system.hh"

// Forward declare reference and pointer parameters
namespace ignition
{
  namespace msgs
  {
    class Plugin_V;
    class StringMsg;
    class Boolean;
  }
}

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class.
    class WorldPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class World World.hh physics/physics.hh
    /// \brief The world provides access to all other object within a simulated
    /// environment.
    ///
    /// The World is the container for all models and their components
    /// (links, joints, sensors, plugins, etc), and WorldPlugin instances.
    /// Many core function are also handled in the World, including physics
    /// update, model updates, and message processing.
    class GZ_PHYSICS_VISIBLE World :
      public boost::enable_shared_from_this<World>
    {
      /// \brief Constructor.
      /// Constructor for the World. Must specify a unique name.
      /// \param[in] _name Name of the world.
      public: explicit World(const std::string &_name = "");

      /// \brief Destructor.
      public: ~World();

      /// \brief Load the world using SDF parameters.
      /// Load a world from and SDF pointer.
      /// \param[in] _sdf SDF parameters.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Get the SDF of the world in the current state.
      /// \return The SDF
      public: const sdf::ElementPtr SDF();

      /// \brief Get the SDF DOM for the world.
      /// \return The SDF DOM for this world.
      public: const sdf::World *GetSDFDom() const;

      /// \brief Save a world to a file.
      /// Save the current world and its state to a file.
      /// \param[in] _filename Name of the file to save into.
      public: void Save(const std::string &_filename);

      /// \brief Initialize the world.
      /// This is called after Load.
      /// \deprecated See Init(UpdateScenePosesFunc)
      public: void Init() GAZEBO_DEPRECATED(11.0);

      /// \brief Initialize the world.
      /// This is called after Load.
      /// \param[in] _func function to be called when Poses are available.
      public: void Init(UpdateScenePosesFunc _func);

      /// \brief Run the world in a thread.
      /// Run the update loop.
      /// \param[in] _iterations Run for this many iterations, then stop.
      /// A value of zero disables run stop.
      public: void Run(const unsigned int _iterations = 0);

      /// \brief Return the running state of the world.
      /// \return True if the world is running.
      public: bool Running() const;

      /// \brief Stop the world.
      /// Request the update loop thread to stop. Wait for it to join if this
      /// function is called from another thread. Return immediately otherwise.
      public: void Stop();

      /// \brief Finalize the world.
      /// Call this function to tear-down the world.
      public: void Fini();

      /// \brief Remove all entities from the world.
      /// This function has delayed effect. Models are cleared at the end
      /// of the current update iteration.
      public: void Clear();

      /// \brief Get the name of the world.
      /// \return The name of the world.
      public: std::string Name() const;

      /// \brief Return the physics engine.
      /// Get a pointer to the physics engine used by the world.
      /// \return Pointer to the physics engine.
      public: PhysicsEnginePtr Physics() const;

      /// \brief Get a reference to the atmosphere model used by the world.
      /// \return Reference to the atmosphere model.
      public: physics::Atmosphere &Atmosphere() const;

      /// \brief Return the preset manager.
      /// \return Pointer to the preset manager.
      public: PresetManagerPtr PresetMgr() const;

      /// \brief Get a reference to the wind used by the world.
      /// \return Reference to the wind.
      public: physics::Wind &Wind() const;

      /// \brief Return the spherical coordinates converter.
      /// \return Pointer to the spherical coordinates converter.
      public: common::SphericalCoordinatesPtr SphericalCoords() const;

      /// \brief Return the gravity vector.
      /// \return The gravity vector.
      public: ignition::math::Vector3d Gravity() const;

      /// \brief Set the gravity vector.
      /// \param[in] _gravity New gravity vector.
      public: void SetGravity(const ignition::math::Vector3d &_gravity);

      /// \brief Set the gravity sdf value.
      /// \param[in] _gravity New gravity vector.
      public: void SetGravitySDF(const ignition::math::Vector3d &_gravity);

      /// \brief Return the magnetic field vector.
      /// \return The magnetic field vector.
      public: ignition::math::Vector3d MagneticField() const;

      /// \brief Set the magnetic field vector.
      /// \param[in] _mag New magnetic field vector.
      public: void SetMagneticField(const ignition::math::Vector3d &_mag);

      /// \brief Get the number of models.
      /// \return The number of models in the World.
      public: unsigned int ModelCount() const;

      /// \brief Get a model based on an index.
      /// Get a Model using an index, where index must be greater than zero
      /// and less than World::ModelCount()
      /// \param[in] _index The index of the model [0..GetModelCount)
      /// \return A pointer to the Model. NULL if _index is invalid.
      public: ModelPtr ModelByIndex(const unsigned int _index) const;

      /// \brief Get a list of all the models.
      /// \return A list of all the Models in the world.
      public: Model_V Models() const;

      /// \brief Get the number of lights.
      /// \return The number of lights in the World.
      public: unsigned int LightCount() const;

      /// \brief Get a list of all the lights.
      /// \return A list of all the Lights in the world.
      public: Light_V Lights() const;

      /// \brief Reset with options.
      /// The _type parameter specifies which type of eneities to reset. See
      /// Base::EntityType.
      /// \param[in] _type The type of reset.
      public: void ResetEntities(Base::EntityType _type = Base::BASE);

      /// \brief Reset simulation time back to zero.
      public: void ResetTime();

      /// \brief Reset time and model poses, configurations in simulation.
      public: void Reset();

      /// \brief Print Entity tree.
      /// Prints alls the entities to stdout.
      public: void PrintEntityTree();

      /// \brief Get the world simulation time, note if you want the PC
      /// wall clock call common::Time::GetWallTime.
      /// \return The current simulation time
      public: common::Time SimTime() const;

      /// \brief Set the sim time.
      /// \param[in] _t The new simulation time
      public: void SetSimTime(const common::Time &_t);

      /// \brief Get the amount of time simulation has been paused.
      /// \return The pause time.
      public: common::Time PauseTime() const;

      /// \brief Get the wall time simulation was started..
      /// \return The start time.
      public: common::Time StartTime() const;

      /// \brief Get the real time (elapsed time).
      /// \return The real time.
      public: common::Time RealTime() const;

      /// \brief Set the initial sim time.
      /// \param[in] _t The new simulation time
      public: void SetInitialSimTime(const common::Time &_t);

      /// \brief Returns the state of the simulation true if paused.
      /// \return True if paused.
      public: bool IsPaused() const;

      /// \brief Set whether the simulation is paused.
      /// \param[in] _p True pauses the simulation. False runs the simulation.
      public: void SetPaused(const bool _p);

      /// \brief Get an element by name.
      /// Searches the list of entities, and return a pointer to the model
      /// with a matching _name.
      /// \param[in] _name The name of the Model to find.
      /// \return A pointer to the entity, or NULL if no entity was found.
      public: BasePtr BaseByName(const std::string &_name) const;

      /// \brief Get a model by name.
      /// This function is the same as BaseByName, but limits the search to
      /// only models.
      /// \param[in] _name The name of the Model to find.
      /// \return A pointer to the Model, or NULL if no model was found.
      public: ModelPtr ModelByName(const std::string &_name) const;

      /// \brief Get a light by name.
      /// This function is the same as BaseByName(), but limits the search to
      /// only lights.
      /// \param[in] _name The name of the Light to find.
      /// \return A pointer to the Light, or NULL if no light was found.
      public: LightPtr LightByName(const std::string &_name) const;

      /// \brief Get a pointer to an Entity based on a name.
      /// This function is the same as BaseByName, but limits the search to
      /// only Entities.
      /// \param[in] _name The name of the Entity to find.
      /// \return A pointer to the Entity, or NULL if no Entity was found.
      public: EntityPtr EntityByName(const std::string &_name) const;

      /// \brief Get the nearest model below and not encapsulating a point.
      /// Only objects below the start point can be returned. Any object
      /// that encapsulates the start point can not be returned from this
      /// function.
      /// This function makes use of World::GetEntityBelowPoint.
      /// \param[in] _pt The 3D point to search below.
      /// \return A pointer to nearest Model, NULL if none is found.
      public: ModelPtr ModelBelowPoint(
                  const ignition::math::Vector3d &_pt) const;

      /// \brief Get the nearest entity below a point.
      /// Projects a Ray down (-Z axis) starting at the given point. The
      /// first entity hit by the Ray is returned.
      /// \param[in] _pt The 3D point to search below
      /// \return A pointer to nearest Entity, NULL if none is found.
      public: EntityPtr EntityBelowPoint(
                  const ignition::math::Vector3d &_pt) const;

      /// \brief Set the current world state.
      /// \param _state The state to set the World to.
      public: void SetState(const WorldState &_state);

      /// \brief Insert a model from an SDF file.
      /// Spawns a model into the world based on an SDF file.
      /// \param[in] _sdfFilename The name of the SDF file (including path).
      public: void InsertModelFile(const std::string &_sdfFilename);

      /// \brief Insert a model from an SDF string.
      /// Spawns a model into the world based on an SDF string.
      /// \param[in] _sdfString A string containing valid SDF markup.
      public: void InsertModelString(const std::string &_sdfString);

      /// \brief Insert a model using SDF.
      /// Spawns a model into the world based on an SDF object.
      /// \param[in] _sdf A reference to an SDF object.
      public: void InsertModelSDF(const sdf::SDF &_sdf);

      /// \brief Return a version of the name with "<world_name>::" removed
      /// \param[in] _name Usually the name of an entity.
      /// \return The stripped world name.
      public: std::string StripWorldName(const std::string &_name) const;

      /// \brief Enable all links in all the models.
      /// Enable is a physics concept. Enabling means that the physics
      /// engine should update an entity.
      public: void EnableAllModels();

      /// \brief Disable all links in all the models.
      /// Disable is a physics concept. Disabling means that the physics
      /// engine should not update an entity.
      public: void DisableAllModels();

      /// \brief Step the world forward in time.
      /// \param[in] _steps The number of steps the World should take.
      public: void Step(const unsigned int _steps);

      /// \brief Load a plugin
      /// \param[in] _filename The filename of the plugin.
      /// \param[in] _name A unique name for the plugin.
      /// \param[in] _sdf The SDF to pass into the plugin.
      public: void LoadPlugin(const std::string &_filename,
                              const std::string &_name,
                              sdf::ElementPtr _sdf);

      /// \brief Remove a running plugin.
      /// \param[in] _name The unique name of the plugin to remove.
      public: void RemovePlugin(const std::string &_name);

      /// \brief Get the set world pose mutex.
      /// \return Reference to the mutex.
      public: std::mutex &WorldPoseMutex() const;

      /// \brief check if physics engine is enabled/disabled.
      /// \param True if the physics engine is enabled.
      public: bool PhysicsEnabled() const;

      /// \brief enable/disable physics engine during World::Update.
      /// \param[in] _enable True to enable the physics engine.
      public: void SetPhysicsEnabled(const bool _enable);

      /// \brief check if wind is enabled/disabled.
      /// \param True if the wind is enabled.
      public: bool WindEnabled() const;

      /// \brief enable/disable wind.
      /// \param[in] _enable True to enable the wind.
      public: void SetWindEnabled(const bool _enable);

      /// \brief check if atmosphere model is enabled/disabled.
      /// \param True if the atmosphere model is enabled.
      public: bool AtmosphereEnabled() const;

      /// \brief enable/disable atmosphere model.
      /// \param[in] _enable True to enable the atmosphere model.
      public: void SetAtmosphereEnabled(const bool _enable);

      /// \brief Update the state SDF value from the current state.
      public: void UpdateStateSDF();

      /// \brief Return true if the world has been loaded.
      /// \return True if World::Load has completed.
      public: bool IsLoaded() const;

      /// \brief Remove all entities from the world. Implementation of
      /// World::Clear
      public: void ClearModels();

      /// \brief Publish pose updates for a model.
      /// This list of models to publish is processed and cleared once every
      /// iteration.
      /// \param[in] _model Pointer to the model to publish.
      public: void PublishModelPose(physics::ModelPtr _model);

      /// \brief Publish scale updates for a model.
      /// This list of models to publish is processed and cleared once every
      /// iteration.
      /// \param[in] _model Pointer to the model to publish.
      public: void PublishModelScale(physics::ModelPtr _model);

      /// \brief Publish pose updates for a light.
      /// Adds light to a list of lights to publish, which is processed and
      /// cleared once every iteration.
      /// \param[in] _light Pointer to the light to publish.
      public: void PublishLightPose(const physics::LightPtr _light);

      /// \brief Get the total number of iterations.
      /// \return Number of iterations that simulation has taken.
      public: uint32_t Iterations() const;

      /// \brief Get the current scene in message form.
      /// \return The scene state as a protobuf message.
      public: msgs::Scene SceneMsg() const;

      /// \brief Run the world. This call blocks.
      /// Run the update loop.
      /// \param[in] _iterations Run for this many iterations, then stop.
      /// A value of zero disables run stop.
      public: void RunBlocking(const unsigned int _iterations = 0);

      /// \brief Remove a model. This function will block until
      /// the physics engine is not locked. The duration of the block
      /// is less than the time to complete a simulation iteration.
      /// \param[in] _model Pointer to a model to remove.
      public: void RemoveModel(ModelPtr _model);

      /// \brief Remove a model by name. This function will block until
      /// the physics engine is not locked. The duration of the block
      /// is less than the time to complete a simulation iteration.
      /// \param[in] _name Name of the model to remove.
      public: void RemoveModel(const std::string &_name);

      /// \brief Reset the velocity, acceleration, force and torque of
      /// all child models.
      public: void ResetPhysicsStates();

      /// \internal
      /// \brief Inform the World that an Entity has moved. The Entity
      /// is added to a list that will be processed by the World.
      /// Only a physics engine implementation should call this function.
      /// If you are unsure whether you should use this function, do not.
      /// \param[in] _entity Entity that has moved.
      public: void _AddDirty(Entity *_entity);

      /// \brief Get whether sensors have been initialized.
      /// \return True if sensors have been initialized.
      public: bool SensorsInitialized() const;

      /// \internal
      /// \brief Set whether sensors have been initialized. This should only
      /// be called by SensorManager.
      /// \param[in] _init True if sensors have been initialized.
      public: void _SetSensorsInitialized(const bool _init);

      /// \brief Return the URI of the world.
      /// \return URI of this world.
      public: common::URI URI() const;

      /// \brief Get a model name which doesn't equal any existing
      /// model's name, by appending numbers to the given name. If _name is
      /// already unique, the returned value is the same.
      /// \param[in] _name Desired model name.
      /// \return Unique model name.
      public: std::string UniqueModelName(const std::string &_name);

      /// \brief Set callback 'waitForSensors'
      /// \param[in] _func function to be called
      public: void SetSensorWaitFunc(std::function<void(double, double)> _func);

      /// \brief Set Visual shininess value by scoped name
      /// \param[in] _scopedName Scoped name of visual.
      /// \param[in] _shininess Shininess value.
      public: void SetVisualShininess(const std::string &_scopedName,
          double _shininess);

      /// \cond
      /// This is an internal function.
      /// \brief Get a model by id.
      /// Each Entity has a unique ID, this function finds a Model with
      /// a passed in _id.
      /// \param[in] _id The id of the Model
      /// \return A pointer to the model, or NULL if no Model was found.
      private: ModelPtr ModelById(const unsigned int _id) const;
      /// \endcond

      /// \brief Load all plugins.
      ///
      /// Load all plugins specified in the SDF for the model.
      private: void LoadPlugins();

      /// \brief Create and load all entities.
      /// \param[in] _sdf SDF element.
      /// \param[in] _parent Parent of the model to load.
      private: void LoadEntities(sdf::ElementPtr _sdf, BasePtr _parent);

      /// \brief Load a model.
      /// \param[in] _sdf SDF element containing the Model description.
      /// \param[in] _parent Parent of the model.
      /// \return Pointer to the newly created Model.
      private: ModelPtr LoadModel(sdf::ElementPtr _sdf, BasePtr _parent);

      /// \brief Load a light.
      /// \param[in] _sdf SDF element containing the Light description.
      /// \param[in] _parent Parent of the light.
      /// \return Pointer to the newly created Light.
      public: LightPtr LoadLight(const sdf::ElementPtr &_sdf,
          const BasePtr &_parent);

      /// \brief Load an actor.
      /// \param[in] _sdf SDF element containing the Actor description.
      /// \param[in] _parent Parent of the Actor.
      /// \return Pointer to the newly created Actor.
      private: ActorPtr LoadActor(sdf::ElementPtr _sdf, BasePtr _parent);

      /// \brief Load a road.
      /// \param[in] _sdf SDF element containing the Road description.
      /// \param[in] _parent Parent of the Road.
      /// \return Pointer to the newly created Road.
      private: RoadPtr LoadRoad(sdf::ElementPtr _sdf, BasePtr _parent);

      /// \brief Function to run physics. Used by physicsThread.
      private: void RunLoop();

      /// \brief Step the world once.
      private: void Step();

      /// \brief Step the world once by reading from a log file.
      private: void LogStep();

      /// \brief Update the world.
      private: void Update();

      /// \brief Pause callback.
      /// \param[in] _p True if paused.
      private: void OnPause(bool _p);

      /// \brief Step callback.
      private: void OnStep();

      /// \brief Called when a world control message is received on the
      /// gazebo_transport topic using boost asio.
      /// \param[in] _data The world control message.
      private: void OnControl(ConstWorldControlPtr &_data);

      /// \brief Called when a world control message is received on the
      /// gz-transport topic using ZeroMQ.
      /// \param[in] _data The world control message.
      private: void OnWorldControl(const msgs::WorldControl &_data);

      /// \brief Called when log playback control message is received.
      /// \param[in] _data The log playback control message.
      private: void OnPlaybackControl(ConstLogPlaybackControlPtr &_data);

      /// \brief Called when a request message is received.
      /// \param[in] _msg The request message.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Construct a scene message from the known world state
      /// \param[out] _scene Scene message to build.
      /// \param[in] _entity Pointer to entity from which to build the scene
      /// message.
      private: void BuildSceneMsg(msgs::Scene &_scene, BasePtr _entity);

      /// \brief Logs joint information.
      /// \param[in] _msg Incoming joint message.
      private: void JointLog(ConstJointPtr &_msg);

      /// \brief Called when a factory message is received.
      /// \param[in] _data The factory message.
      private: void OnFactoryMsg(ConstFactoryPtr &_data);

      /// \brief Called when a model message is received.
      /// \param[in] _msg The model message.
      private: void OnModelMsg(ConstModelPtr &_msg);

      /// \brief TBB version of model updating.
      private: void ModelUpdateTBB();

      /// \brief Single loop version of model updating.
      private: void ModelUpdateSingleLoop();

      /// \brief Helper function to load a plugin from SDF.
      /// \param[in] _sdf SDF plugin description.
      private: void LoadPlugin(sdf::ElementPtr _sdf);

      /// \brief Fills a model message with data from a model
      /// \param[out] _msg Model message to fill.
      /// \param[in] _model Pointer to the model to get the data from.
      private: void FillModelMsg(msgs::Model &_msg, ModelPtr _model);

      /// \brief Process all received entity messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessEntityMsgs();

      /// \brief Process all received request messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessRequestMsgs();

      /// \brief Process all received factory messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessFactoryMsgs();

      /// \brief Process all received model messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessModelMsgs();

      /// \brief Process all received light factory messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessLightFactoryMsgs();

      /// \brief Process all received light modify messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessLightModifyMsgs();

      /// \brief Process all received log playback control messages.
      /// Must only be called from the World::ProcessMessages function.
      private: void ProcessPlaybackControlMsgs();

      /// \brief Log callback. This is where we write out state info.
      private: bool OnLog(std::ostringstream &_stream);

      /// \brief Save model resources while recording a log, such as meshes
      /// and textures.
      private: void LogModelResources();

      /// \brief Process all incoming messages.
      private: void ProcessMessages();

      /// \brief Publish the world stats message.
      private: void PublishWorldStats();

      /// \brief Thread function for logging state data.
      private: void LogWorker();

      /// \brief Register items in the introspection service.
      private: void RegisterIntrospectionItems();

      /// \brief Unregister items in the introspection service.
      private: void UnregisterIntrospectionItems();

      /// \brief Callback when a light message is received in the
      /// ~/factory/light topic.
      /// \param[in] _msg Pointer to the light message.
      private: void OnLightFactoryMsg(ConstLightPtr &_msg);

      /// \brief Callback when a light message is received in the
      /// ~/light/modify topic.
      /// \param[in] _msg Pointer to the light message.
      private: void OnLightModifyMsg(ConstLightPtr &_msg);

      /// \brief Callback for "<this_name>/physics/info/plugin" service.
      /// Get information about plugins in this world or one of its
      /// children, according to the given _pluginUri. Some _pluginUri examples:
      ///
      /// * Info about a specific world plugin in this world:
      ///    data://world/<this_name>/plugin/<plugin_name>
      ///
      /// * Info about all world plugins in this world (empty plugin name):
      ///    data://world/<this_name>/plugin
      ///
      /// * Info about a model plugin in a child model:
      ///    data://world/<this_name>/model/<model_name>/plugin/<plugin_name>
      ///
      /// \param[in] _request Request containing plugin URI.
      /// \param[out] _plugins Message containing vector of plugins.
      /// \return True if the info was successfully obtained.
      private: bool PluginInfoService(const ignition::msgs::StringMsg &_request,
          ignition::msgs::Plugin_V &_plugins);

      /// \brief Callback for "<this_name>/scene_info" service.
      /// \param[out] _response Message containing scene info
      /// \return True if the info was successfully obtained.
      private: bool SceneInfoService(msgs::Scene &_response);

      /// \brief Callback for "<this_name>/shadow_caster_material_name" service.
      /// \param[out] _response Message containing shadow caster material name
      /// \return True if the info was successfully obtained.
      private: bool ShadowCasterMaterialNameService(
          ignition::msgs::StringMsg &_response);

      /// \brief Callback for "<this_name>/spherical_coordinates_surface_type" service.
      /// \param[out] _response Message containing spherical coordinates surface type.
      /// \return True if the info was successfully obtained.
      public: bool SphericalCoordinatesSurfaceService(
          ignition::msgs::StringMsg &_response);

      /// \brief Callback for "<this_name>/shadow_caster_render_back_faces"
      ///     service.
      /// \param[out] _response Message containing shadow caster render back
      ///     faces
      /// \return True if the info was successfully obtained.
      private: bool ShadowCasterRenderBackFacesService(
          ignition::msgs::Boolean &_response);

      /// \brief Callback for "<model_name>/shininess" service.
      /// \param[in] _request Message containing the model name.
      /// \param[out] _response Message containing shininess value.
      /// \return True if the info was successfully obtained.
      private: bool MaterialShininessService(
          const ignition::msgs::StringMsg &_request, msgs::Any &_response);

      /// \brief Helper function for getting shininess values by scoped
      /// visual name.
      /// \param[in] _scopedName Scoped visual name.
      /// \return Shininess value.
      private: double ShininessByScopedName(const std::string &_scopedName)
          const;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<WorldPrivate> dataPtr;

      /// Friend DARTLink so that it has access to dataPtr->dirtyPoses
      private: friend class DARTLink;

      /// Friend SimbodyPhysics so that it has access to dataPtr->dirtyPoses
      private: friend class SimbodyPhysics;
    };
    /// \}
  }
}
#endif
