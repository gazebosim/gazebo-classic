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
 * SVN: $Id:$
 */

#ifndef WORLD_HH
#define WORLD_HH

#include <vector>
#include "Vector3.hh"
#include "Pose3d.hh"

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief The World
/// \{

// Forward declarations
  class Server;
  class SimulationIface;
  class Model;
  class PhysicsEngine;
  class XMLConfigNode;
  class XMLConfig;

/// \brief The World
/*
 * The world class keps a list of all models, handles loading and saving,
 * object dynamics and collision detection for contact joints
 */
class World
{
  /// Private constructor
  private: World();

  /// Private destructor
  public: ~World();

  /// Get an instance of this World
  /// \return Instance of the World
  public: static World *Instance();

  /// Load the world
  /// \param node XMLConfig node point
  /// \param serverId Id of the gazebo server
  public: void Load(XMLConfig *node, int serverId);

  /// Initialize the world
  /// \return 0 on success
  public: int Init();

  /// Update the world
  /// \return 0 on success
  public: int Update();

  /// Finilize the world
  /// \return 0 on success
  public: int Fini();

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
  /// \return 0 on success
  public: int LoadEntities(XMLConfigNode *node, Model *parent);

  /// \brief Delete an entity by name
  /// \param name The name of the entity to delete
  public: void DeleteEntity(const char *name);

  /// \brief Load a model
  /// \param node Pointer to the XMLConfig node
  /// \param parent The parent model
  /// \return The model that was created
  private: Model *LoadModel(XMLConfigNode *node, Model *parent);

  /// \brief Set the model pose and the pose of it's attached children 
  /// \param model The model to set
  /// \param pose The pose to set the model to
  private: void SetModelPose(Model *model , Pose3d pose);

  /// Pointer to myself
  private: static World *myself;

  /// Pointer the physics engine
  private: PhysicsEngine *physicsEngine;

  /// List of all the models
  private: std::vector< Model* > models;

  /// List of models to add into the world
  private: std::vector< Model* > toAddModels;

  /// List of models to delete from the world
  private: std::vector< Model* > toDeleteModels;

  /// Simulator control interface
  private: Server *server;

  /// Simulation interface
  private: SimulationIface *simIface;

  /// Flag set if simulation is paused
  private: bool pause;

  /// Current simulation time
  private: double simTime, pauseTime, startTime;

};


/// \}
}

#endif
