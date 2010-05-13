/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#ifndef WORLD_HH
#define WORLD_HH

#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <string>

#include <boost/tuple/tuple.hpp>
#include <boost/signal.hpp>

#include "Global.hh"

#ifdef USE_THREADPOOL
#include "boost/threadpool.hpp"
#include "boost/thread/mutex.hpp"
#endif

#include "SingletonT.hh"
#include "Vector3.hh"
#include "Pose3d.hh"
#include "Entity.hh"
#include "Timer.hh"

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief The World
/// \{

// Forward declarations
  class Server;
  class SimulationIface;
  class Model;
  class Geom;
  class PhysicsEngine;
  class XMLConfigNode;
  class GraphicsIfaceHandler;
  class OpenAL;
  class Factory;
  class WorldState;
  class Timer;
  class Time;
   
/// \brief The World
/*
 * The world class keps a list of all models, handles loading and saving,
 * object dynamics and collision detection for contact joints
 */
class World : public SingletonT<World>
{
  /// Private constructor
  private: World();

  /// Private destructor
  private: ~World();

  ///Closes the present world, frees the resources and closes the interfaces   
  public: void Close();

  /// Load the world
  /// \param node XMLConfig node point
  /// \param serverId Id of the gazebo server
  public: void Load(XMLConfigNode *rootNode, unsigned int serverId);

  /// Save the world
  /// \param stream Output stream
  public: void Save(std::string &prefix, std::ostream &stream);

  /// Initialize the world
  public: void Init();

  /// Update the world
  public: void Update();

  /// \brief Primarily used to update the graphics interfaces
  public: void GraphicsUpdate();

  /// Finilize the world
  public: void Fini();

  /// Retun the libgazebo server
  /// \return Pointer the the libgazebo server
  public: Server *GetGzServer() const;

  /// Return the physics engine
  /// \return Pointer to the physics engine
  public: PhysicsEngine *GetPhysicsEngine() const;

  /// Get the simulation time
  /// \return The simulation time
  public: double GetSimTime() const;

  /// Get the pause time
  /// \return The pause time
  public: double GetPauseTime() const;

  /// Get the start time
  /// \return The start time
  public: double GetStartTime() const;

  /// Get the real time (elapsed time)
  /// \return The real time
  public: double GetRealTime() const;

  /// \brief Get the wall clock time
  /// \return The wall clock time
  public: double GetWallTime() const;

  /// \brief Load all entities
  /// \param node XMLConfg node pointer
  /// \param parent Parent of the model to load
  /// \param removeDuplicate Remove existing model of same name
  public: void LoadEntities(XMLConfigNode *node, Model *parent, 
                            bool removeDuplicate,bool initModel);

  /// \brief Insert an entity into the world. This function pushes the model
  //  (encoded as an XML string) onto a list. The Graphics Thread will then
  //  call the ProcessEntitiesToLoad function to actually make the new
  //  entities. This Producer-Consumer model is necessary for thread safety.
  public: void InsertEntity(std::string xmlString);

  /// \brief Load all the entities that have been queued
  public: void ProcessEntitiesToLoad();

  /// \brief Delete all the entities that have been queued
  public: void ProcessEntitiesToDelete();

  /// \brief Delete an entity by name
  /// \param name The name of the entity to delete
  public: void DeleteEntity(const std::string &name);

  /// \brief Get a pointer to a entity based on a name
  public: Entity *GetEntityByName(const std::string &name) const;
  private: Entity *GetEntityByNameHelper(const std::string &name, Entity *parent) const;

  /// \brief Get an iterator over the models
  public: const std::vector<Model*> &GetModels() const;

  /// \brief Reset the simulation to the initial settings
  public: void Reset();

  /// \brief register a geom
  public: void RegisterGeom(Geom *geom);

  /// \brief Register a body 
  public: void RegisterBody(Body *body);

  // User control of how the world is viewed 
  // If this section grows it may become a model-view structure ...
  /// \brief Return true if the bounding boxes should be shown
  public: bool GetShowBoundingBoxes();

  /// \brief Set if the bounding boxes should be shown
  public: void SetShowBoundingBoxes(bool show);

  /// \brief Get wheter to show the joints
  public: bool GetShowJoints();

  /// \brief Set whether to show the joints
  public: void SetShowJoints(bool show);

  /// \brief Get whether to show the joints
  public: bool GetShowJoints() const;

  /// \brief Set whether to show the contacts
  public: void SetShowContacts(bool show);

  /// \brief Get whether to show the contacts
  public: bool GetShowContacts() const;

  /// \brief Set whether to show the light source visuals
  public: void SetShowLights(bool show);

  /// \brief Get whether to show the light source visuals
  public: bool GetShowLights() const;

  /// \brief Set whether to show the camera visuals
  public: void SetShowCameras(bool show);

  /// \brief Get whether to show the camera visuals
  public: bool GetShowCameras() const;

  /// \brief Set to view as wireframe
  public: void SetWireframe( bool wire );

  /// \brief Get whether to view as wireframe
  public: bool GetWireframe();

  /// \brief Set to view as wireframe
  public: void SetShowPhysics( bool show );

  /// \brief Get whether to view as wireframe
  public: bool GetShowPhysics();

  /// \brief Goto a position in time
  public: void GotoTime(double pos);

  /// \brief Set the selected entity
  public: void SetSelectedEntity( Entity *ent );

  /// \brief Get the selected entity
  public: Entity *GetSelectedEntity() const;

  /// \brief Save the state of the world
  private: void SaveState();

  /// \breif Set the state of the world to the pos pointed to by the iterator
  private: void SetState(std::deque<WorldState>::iterator iter);

  /// \brief Pause callback
  private: void PauseSlot(bool p);

  /// Set to true to show bounding boxes
  private: bool showBoundingBoxes;

  /// Set to true to show joints
  private: bool showJoints;

  private: bool showContacts;

  private: bool showLights;

  private: bool showCameras;

  private: bool showPhysics;

  private: bool wireframe;


  /// \brief Load a model
  /// \param node Pointer to the XMLConfig node
  /// \param parent The parent model
  /// \param removeDuplicate Remove existing model of same name
  /// \return The model that was created
  private: Model *LoadModel(XMLConfigNode *node, Model *parent, bool removeDuplicate,bool initModel);

  /// \brief Update the simulation iface
  public: void UpdateSimulationIface();

  /// \brief Connect a boost::slot the the add entity signal
  public: template<typename T>
          boost::signals::connection ConnectAddEntitySignal( T subscriber )
          { return addEntitySignal.connect(subscriber); }

  public: template<typename T>
          void DisconnectAddEntitySignal( T subscriber)
          { addEntitySignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the show light source signal
  public: template<typename T>
          boost::signals::connection ConnectShowLightsSignal( T subscriber )
          { return showLightsSignal.connect(subscriber); }

  public: template<typename T>
          void DisconnectShowLightsSignal( T subscriber )
          { showLightsSignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the show camera source signal
  public: template<typename T>
          boost::signals::connection ConnectShowCamerasSignal( T subscriber )
          { return showCamerasSignal.connect(subscriber); }
  public: template<typename T>
          void DisconnectShowCamerasSignal( T subscriber )
          { showCamerasSignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the show contacts signal
  public: template<typename T>
          boost::signals::connection ConnectShowContactsSignal( T subscriber )
          { return showContactsSignal.connect(subscriber); }
  public: template<typename T>
          void DisconnectShowContactsSignal( T subscriber )
          { showContactsSignal.disconnect(subscriber); }


  /// \brief Connect a boost::slot the the show wireframe signal
  public: template<typename T>
          boost::signals::connection ConnectShowWireframeSignal( T subscriber )
          { return wireframeSignal.connect(subscriber); }
  public: template<typename T>
          void DisconnectShowWireframeSignal( T subscriber )
          { wireframeSignal.disconnect(subscriber); }


  /// \brief Connect a boost::slot the the show physics signal
  public: template<typename T>
          boost::signals::connection ConnectShowPhysicsSignal( T subscriber )
          { return showPhysicsSignal.connect(subscriber); }
  public: template<typename T>
          void DisconnectShowPhysicsSignal( T subscriber )
          { showPhysicsSignal.disconnect(subscriber); }


  /// \brief Connect a boost::slot the the show joints signal
  public: template<typename T>
          boost::signals::connection ConnectShowJointsSignal( T subscriber )
          { return showJointsSignal.connect(subscriber); }
  public: template<typename T>
          void DisconnectShowJointsSignal( T subscriber )
          { showJointsSignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the show bounding boxes signal
  public: template<typename T>
          boost::signals::connection ConnectShowBoundingBoxesSignal( T subscriber )
          { return showBoundingBoxesSignal.connect(subscriber); }
  public: template<typename T>
          void DisconnectShowBoundingBoxesSignal( T subscriber )
          { showBoundingBoxesSignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the world update start signal
  public: template<typename T>
          boost::signals::connection ConnectWorldUpdateStartSignal(T subscriber)
          { return worldUpdateStartSignal.connect(subscriber); }
  /// \brief Disconnect a boost::slot the the world update start signal
  public: template<typename T>
          void DisconnectWorldUpdateStartSignal( T subscriber )
          { worldUpdateStartSignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the world update end signal
  public: template<typename T>
          boost::signals::connection ConnectWorldUpdateEndSignal(T subscriber)
          { return worldUpdateEndSignal.connect(subscriber); }
  /// \brief Disconnect a boost::slot the the world update end signal
  public: template<typename T>
          void DisconnectWorldUpdateEndSignal( T subscriber )
          { worldUpdateEndSignal.disconnect(subscriber); }

  /// \brief Connect a boost::slot the the entity selected signal
  public: template<typename T>
          boost::signals::connection ConnectEntitySelectedSignal(T subscriber)
          { return entitySelectedSignal.connect(subscriber); }
  /// \brief Disconnect a boost::slot the the entity selected signal
  public: template<typename T>
          void DisconnectEntitySelectedSignal( T subscriber )
          { entitySelectedSignal.disconnect(subscriber); }

  /// \brief Get the names of interfaces defined in the tree of a model
  private: void GetInterfaceNames(Entity* m, std::vector<std::string>& list);

  /// Pointer the physics engine
  private: PhysicsEngine *physicsEngine;

  /// An abstract entity that is the root of the Entity Tree
  private: std::vector<Model*> models;

  /// List of models to delete from the world
  private: std::vector< std::string > toDeleteEntities;

  private: std::vector< std::string > toLoadEntities;

  /// Simulator control interface
  private: Server *server;

  /// Simulation interface
  private: SimulationIface *simIface;

  private: Factory *factory;

  private: GraphicsIfaceHandler *graphics;

  /// Length of time to run before receiving a "go" command
  private: Time simPauseTime;

  private: OpenAL *openAL;

  /// List of all the parameters
  protected: std::vector<Param*> parameters;

  private: ParamT<int>* threadsP;

#ifdef USE_THREADPOOL
  public: boost::threadpool::pool* threadPool;
#endif

  private: friend class DestroyerT<World>;
  private: friend class SingletonT<World>;

  /// The entity currently selected by the user
  private: Entity *selectedEntity;


  private: boost::signal<void (Entity*)> addEntitySignal;
  private: boost::signal<void (bool)> showLightsSignal;
  private: boost::signal<void (bool)> showCamerasSignal;
  private: boost::signal<void (bool)> showContactsSignal;
  private: boost::signal<void (bool)> wireframeSignal;
  private: boost::signal<void (bool)> showPhysicsSignal;
  private: boost::signal<void (bool)> showJointsSignal;
  private: boost::signal<void (bool)> showBoundingBoxesSignal;
  private: boost::signal<void (Entity*)> entitySelectedSignal;

  private: boost::signal<void ()> worldUpdateStartSignal;
  private: boost::signal<void ()> worldUpdateEndSignal;

  private: std::deque<WorldState> worldStates;
  private: std::deque<WorldState>::iterator worldStatesInsertIter;
  private: std::deque<WorldState>::iterator worldStatesEndIter;
  private: std::deque<WorldState>::iterator worldStatesCurrentIter;

  private: Timer saveStateTimer;
  private: ParamT<Time> *saveStateTimeoutP;
  private: ParamT<unsigned int> *saveStateBufferSizeP;
};

class WorldState
{
  public: std::map<std::string, Pose3d> modelPoses;
  public: std::map<std::string, Pose3d> bodyPoses;
  public: std::map<std::string, Pose3d> geomPoses;
};

/// \}
}

#endif
