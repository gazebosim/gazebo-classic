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

#include "Param.hh"
#include "PhysicsEngine.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

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

\par Attributes

- cfm (float)
  - Global constraint force mixing
  - Default: 10e-5
  - Range:  10e-10 to 1.0
  - Recommended value: 10e-5
- erp (float)
  - Global error reduction parameter
  - Default: 0.2
  - Range: 0 to 1.0
  - Recommended Range: 0.1 to 0.8
- stepTime (float)
  - Time, in seconds, that elapse for each iteration of the physics engine
  - Default: 0.025
-gravity (float float float)
  - Gravity vector.
  - Default: 0 0 -9.8

\verbatim
<physics:ode>
  <stepTime>0.03</stepTime>
  <gravity>0 0 -9.8</gravity>
  <cfm>10e-5</cfm>
  <erp>0.2</erp>
</physcis:ode>
\endverbatim


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
  public: void Save(std::string &prefix, std::ostream &stream);

  /// \brief Initialize the ODE engine
  public: virtual void Init();

  /// \brief Initialize for separate thread
  public: virtual void InitForThread();

  /// \brief Update the ODE collision
  public: virtual void UpdateCollision();

  /// \brief Update the ODE engine
  public: virtual void UpdatePhysics();

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

  /// \brief Get the world id
  public: dWorldID GetWorldId();

  /// \brief Do collision detection
  private: static void CollisionCallback( void *data, dGeomID o1, dGeomID o2);

  /// \brief Top-level world for all bodies
  private: dWorldID worldId;

  /// \brief Top-level space for all sub-spaces/geoms
  private: dSpaceID spaceId;

  /// \brief Collision attributes
  private: dJointGroupID contactGroup;

  private: ParamT<double> *globalCFMP; 
  private: ParamT<double> *globalERPP; 
  private: ParamT<bool> *quickStepP; 
  private: ParamT<int> *quickStepItersP; 
  private: ParamT<double> *quickStepWP; 
  private: ParamT<double> *contactMaxCorrectingVelP;
  private: ParamT<double> *contactSurfaceLayerP;
};

/** \}*/
/// \}
}

#endif
