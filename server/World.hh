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
#include <boost/tuple/tuple.hpp>

#include "SingletonT.hh"
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
  class Geom;
  class PhysicsEngine;
  class XMLConfigNode;
  class GraphicsIfaceHandler;
  class OpenAL;
   
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
                            bool removeDuplicate);

  /// \brief Insert an entity into the world. This function pushes the model
  //  (encoded as an XML string) onto a list. The Graphics Thread will then
  //  call the ProcessEntitiesToLoad function to actually make the new
  //  entities. This Producer-Consumer model is necessary for thread safety.
  public: void InsertEntity(std::string xmlString);

  /// \brief Load all the entities that have been queued
  public: void ProcessEntitiesToLoad();

  /// \brief Delete an entity by name
  /// \param name The name of the entity to delete
  public: void DeleteEntity(const char *name);

  /// \brief Get a pointer to a model based on a name
  public: Model *GetModelByName(std::string modelName);

  /// \brief Get an iterator over the models
  public: std::vector<Model*> &GetModels();

  /// \brief Reset the simulation to the initial settings
  public: void Reset();

  /// \brief register a geom This method is no more than a manually-done signal system
  public: void RegisterGeom(Geom *geom);

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

  /// \brief Set to view as wireframe
  public: void SetWireframe( bool wire );

  /// \brief Get whether to view as wireframe
  public: bool GetWireframe();

  /// \brief Set to view as wireframe
  public: void SetShowPhysics( bool show );

  /// \brief Get whether to view as wireframe
  public: bool GetShowPhysics();

  public: void ProcessMessages();

  /// Set to true to show bounding boxes
  private: bool showBoundingBoxes;

  /// Set to true to show joints
  private: bool showJoints;

  private: bool showPhysics;

  private: bool wireframe;


  /// \brief Load a model
  /// \param node Pointer to the XMLConfig node
  /// \param parent The parent model
  /// \param removeDuplicate Remove existing model of same name
  /// \return The model that was created
  private: Model *LoadModel(XMLConfigNode *node, Model *parent, bool removeDuplicate);

  /// \brief Set the model pose and the pose of it's attached children 
  /// \param model The model to set
  /// \param pose The pose to set the model to
  private: void SetModelPose(Model *model , Pose3d pose);

  /// \brief Update the simulation iface
  private: void UpdateSimulationIface();

  /// Pointer the physics engine
  private: PhysicsEngine *physicsEngine;

  /// List of all the models
  private: std::vector< Model* > models;

  /// List of all the registered geometries
  private: std::vector< Geom* > geometries;

  /// List of models to add into the world
  private: std::vector< Model* > toAddModels;

  /// List of models to delete from the world
  private: std::vector< Model* > toDeleteModels;

  private: std::vector< std::string > toLoadEntities;

  /// Simulator control interface
  private: Server *server;

  /// Simulation interface
  private: SimulationIface *simIface;
  private: GraphicsIfaceHandler *graphics;

  /// Length of time to run before receiving a "go" command
  private: double simPauseTime;

  private: OpenAL *openAL;

  private: friend class DestroyerT<World>;
  private: friend class SingletonT<World>;

};


/// \}
}

#endif
