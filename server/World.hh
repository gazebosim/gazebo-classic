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

#include <vector>

// Forward declarations
class Server;
class SimulationIface;

namespace gazebo
{
  class Model;
  class PhysicsEngine;
  class XMLConfigNode;
  class XMLConfig;


/// @addtogroup gazebocore
/// @{

/** @defgroup models_global Global Attributes

The @c param:Global tag is used to specify certain global parameters
for the server. Set these parameters as needed in the world file E.g.:

@verbatim
<param:Global>
  <gravity>0.0 0.0 -9.8 </gravity>
  <stepTime>0.020</stepTime>
</param:Global>
@endverbatim

The parameters are as follows:

- speed (float)
  - Target simulation speed (e.g. speed 2 yields twice real time).
  - Default 1.0

- gravity (float vector)
  - The gravity vector (m/sec/sec); the default corresponds to Earth gravity.
  - Default 0 0 -9.8

- stepTime (float)
  - The minimum step time for the simulator.  Reducing the step time
    will increase the fidelity of the physical simulation, but consume
    more CPU time.  If you have particulary complex system that appears to
    be diverging (i.e., objects "explode" when they come into collision), consider
    reducing the step time.
  - Default 0.020

- pause (bool)
  - Set to true to start the simulator in paused mode.
  - Default: false

- dayTime (float)
  - Real time seconds since 12am. Negative number for local time
  - Default: -1.0

- utmZone (integer)
  - UTM (Universal Transverse Mercator) zone for calculating latitudes
    and longitudes from UTM easting and northings.
  - Default 0

- utmOffset (float vector)
  - UTM value (easting and northings) of world origin (used for
    calculating object UTM coordinates).  The z-value can also be used
    to specify an altitude offset.
  - Default 0

- skyColor (float vector) 
  - The color of the "sky", i.e., the default background color.
  - Default 0.7 0.7 0.9

- fogDensity (float)
  - Density of the ambient fog (see OpenGL docs for details).
  - Default 0.25

*/

/// The World
/*
 * The world class keps a list of all models, handles loading and saving,
 * object dynamics and collision detection for contact joints
 */
class World
{
  /// Private constructor
  private: World();

  /// Private destructor
  private: ~World();

  /// Get an instance of this World
  /// \return Instance of the World
  public: static World *Instance();

  /// Load the world
  /// \param node XMLConfig node point
  /// \param serverId Id of the gazebo server
  public: int Load(XMLConfig *node, int serverId);

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

  /// Load all models
  /// \param node XMLConfg node pointer
  /// \param parent Parent of the model to load
  /// \return 0 on success
  private: int LoadModel(XMLConfigNode *node, Model *parent);

  /// Pointer to myself
  private: static World *myself;

  /// Pointer the physics engine
  private: PhysicsEngine *physicsEngine;

  /// List of all the models
  private: std::vector<Model*> models;

  /// Simulator control interface
  private: Server *server;

  /// Simulation interface
  private: SimulationIface *simIface;

  /// Flag set if simulation is paused
  private: bool pause;
};

/// @}

}

#endif
