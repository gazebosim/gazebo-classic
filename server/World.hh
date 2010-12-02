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

#include "SingletonT.hh"
#include "Vector3.hh"
#include "Pose3d.hh"
#include "Entity.hh"
#include "Timer.hh"

namespace libgazebo
{
  class Server;
}

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief The World
/// \{

// Forward declarations
  class Model;
  class Body;
  class Geom;
  class PhysicsEngine;
  class XMLConfigNode;
  class OpenAL;
  class Factory;
  class WorldState;
  class Timer;
  class Time;
  class GraphicsIfaceHandler;
  class SimulationIfaceHandler;
   
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

  /// \brief Remove all entities from the world
  public: void Clear();

  /// \brief Get the number of parameters
  public: unsigned int GetParamCount() const;

  /// \brief Get a param
  public: Param *GetParam(unsigned int index) const;

  /// Retun the libgazebo server
  /// \return Pointer the the libgazebo server
  public: libgazebo::Server *GetGzServer() const;

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

  /// \brief Get the number of models
  public: unsigned int GetModelCount() const;

  /// \brief Get a model based on an index
  public: Model *GetModel(unsigned int index);

  /// \brief Reset the simulation to the initial settings
  public: void Reset();

  /// \brief register a geom
  public: void RegisterGeom(Geom *geom);

  /// \brief Register a body 
  public: void RegisterBody(Body *body);

  /// \brief Goto a position in time
  public: void GotoTime(double pos);

  /// \brief Get the selected entity
  public: Entity *GetSelectedEntity() const;

  /// \brief Print entity tree
  public: void PrintEntityTree();

  /// \brief Get the server id
  public: int GetServerId() const;

  /// \brief Save the state of the world
  private: void SaveState();

  /// \breif Set the state of the world to the pos pointed to by the iterator
  private: void SetState(std::deque<WorldState>::iterator iter);

  /// \brief Pause callback
  private: void PauseSlot(bool p);


  /// \brief Load a model
  /// \param node Pointer to the XMLConfig node
  /// \param parent The parent model
  /// \param removeDuplicate Remove existing model of same name
  /// \return The model that was created
  private: Model *LoadModel(XMLConfigNode *node, Model *parent, bool removeDuplicate,bool initModel);

  /// \brief Update the simulation iface
  public: void UpdateSimulationIface();

  /// \brief Delete an entity by name
  /// \param name The name of the entity to delete
  private: void DeleteEntityCB(const std::string &name);

  /// \brief Set the selected entity
  private: void SetSelectedEntityCB( const std::string &name );

  /// Pointer the physics engine
  private: PhysicsEngine *physicsEngine;

  /// An abstract entity that is the root of the Entity Tree
  private: std::vector<Model*> models;

  /// List of models to delete from the world
  private: std::vector< std::string > toDeleteEntities;

  private: std::vector< std::string > toLoadEntities;

  /// Simulator control interface
  private: libgazebo::Server *server;

  private: Factory *factory;

  private: GraphicsIfaceHandler *graphics;
  private: SimulationIfaceHandler *simIfaceHandler;

  private: OpenAL *openAL;

  /// List of all the parameters
  protected: std::vector<Param*> parameters;

  private: ParamT<int>* threadsP;

  private: friend class DestroyerT<World>;
  private: friend class SingletonT<World>;

  /// The entity currently selected by the user
  private: Entity *selectedEntity;

  public: double contactMarkerSize;

  //  private: boost::signal<void (Entity*)> addEntitySignal;
  //  private: boost::signal<void (std::string)> deleteEntitySignal;
  //  private: boost::signal<void (bool)> showLightsSignal;
  //  private: boost::signal<void (bool)> showCamerasSignal;
  //  private: boost::signal<void (bool)> showContactsSignal;
  //  private: boost::signal<void (bool)> wireframeSignal;
  //  private: boost::signal<void (bool)> showPhysicsSignal;
  //  private: boost::signal<void (bool)> showJointsSignal;
  //  private: boost::signal<void (bool)> showBoundingBoxesSignal;
  //  private: boost::signal<void (Entity*)> entitySelectedSignal;
  // 
  //  private: boost::signal<void ()> worldUpdateStartSignal;
  //  private: boost::signal<void ()> worldUpdateEndSignal;

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
