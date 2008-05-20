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
/* Desc: The ODE physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 * SVN: $Id$
 */

#ifndef ODEPHYSICS_HH
#define ODEPHYSICS_HH

#include <ode/ode.h>
#include <map>

#include "PhysicsEngine.hh"


namespace gazebo
{
  class SphereGeom;
  class PlaneGeom;
  class BoxGeom;
  class CylinderGeom;
  class Entity;
  class XMLConfigNode;
/// \addtogroup gazebo_physics_engine
/// \{

/** \defgroup odephysicsengine ODE Physics Engine

The \c param:physics tag is used to specify certain parameters for the ODE phyics engine. The following parameters are in addition to those provided by the PhysicsEngine base class.

\verbatim
<param:physics>
  <cfm>10e-9</cfm>
  <erp>0.2</erp>
</param:physics>
\endverbatim

- cfm (float)
  - Global Constraint Force Mixing parameter
  - Range:  10e-10 to 1.0
  - Recommended value: 10e-9

- erp (float)
  - Global Error Reduction parameter
  - Range: 0 to 1.0
  - Recommended Range: 0.1 to 0.8

\{
*/

/// \brief ODE physics engine
class ODEPhysics : public PhysicsEngine
{
  /// \brief Constructor
  public: ODEPhysics();

  /// \brief Destructor
  public: virtual ~ODEPhysics();

  /// \brief Load the ODE engine
  public: virtual void Load(XMLConfigNode *node);

  /// \brief Saves to XMLFile
  public: void Save(XMLConfigNode *node);

  /// \brief Initialize the ODE engine
  public: virtual void Init();

  /// \brief Update the ODE engine
  public: virtual void Update();

  /// \brief Finilize the ODE engine
  public: virtual void Fini();

  /// \brief Add an entity
  public: virtual void AddEntity(Entity *entity);

  /// \brief Remove an entity from the physics engine
  public: virtual void RemoveEntity(Entity *entity);

  /// \brief Create a new body
  public: virtual Body *CreateBody(Entity *parent);

  /// \brief Create a new joint
  public: virtual Joint *CreateJoint(Joint::Type type);

  /// \brief Return the space id 
  public: dSpaceID GetSpaceId() const;

  /// \brief Do collision detection
  private: static void CollisionCallback( void *data, dGeomID o1, dGeomID o2);

  /// \brief Top-level world for all bodies
  private: dWorldID worldId;

  /// \brief Top-level space for all sub-spaces/geoms
  private: dSpaceID spaceId;

  /// \brief Collision attributes
  private: dJointGroupID contactGroup;
          
  /// List of all the entities
  private: std::map<int, Entity* > entities;

  private: double globalCFM;
  private: double globalERP; 
};

/** \}*/
/// \}
}

#endif
