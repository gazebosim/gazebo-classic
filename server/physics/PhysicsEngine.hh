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
/* Desc: The base class for all physics engines
 * Author: Nate Koenig
 * Date: 11 June 2007
 * SVN: $Id$
 */

#ifndef PHYSICSENGINE_HH
#define PHYSICSENGINE_HH

#include "Joint.hh"

namespace gazebo
{

class Entity;
class Body;
class XMLConfigNode;

/// \addtogroup gazebo_physics_engine
/** \{

\verbatim
<physics:<engine_type>>
  <gravity>0.0 0.0 -9.8 </gravity>
  <stepTime>0.020</stepTime>
</physics:<engine_type>>
\endverbatim

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

*/

/// \brief Base class for a physics engine
class PhysicsEngine
{
  /// \brief Default constructor
  public: PhysicsEngine();

  /// \brief Destructor
  public: virtual ~PhysicsEngine();

  /// \brief Load the physics engine
  /// \param node Pointer to the XML parameters
  public: virtual void Load(XMLConfigNode *node) = 0;

  /// \brief Initialize the physics engine
  public: virtual void Init() = 0;

  /// \brief Update the physics engine
  public: virtual void Update() = 0;

  /// \brief Finilize the physics engine
  public: virtual void Fini() = 0;

  /// \brief Add an entity
  public: virtual void AddEntity(Entity *entity) = 0;

  /// \brief Remove an entity from the physics engine
  public: virtual void RemoveEntity(Entity *entity) = 0;

  /// \brief Create a new body
  public: virtual Body *CreateBody(Entity *parent) = 0;

  /// \brief Create a new joint
  public: virtual Joint *CreateJoint(Joint::Type type) = 0;

  /// \brief Return the gavity vector
  /// \return The gavity vector
  public: Vector3 GetGravity() const;

  /// \brief Get the time between each update cycle
  /// \return Time in seconds
  public: double GetStepTime() const;

  /// The gravity vector
  protected: Vector3 gravity;

  /// Time between each update cycle
  protected: double stepTime;
};

/** \}*/
}
#endif
