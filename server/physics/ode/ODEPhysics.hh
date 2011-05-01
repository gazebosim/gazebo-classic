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

#include "gazebo_config.h"

#include "Param.hh"
#include "PhysicsEngine.hh"
#include "Shape.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#ifdef PARALLEL_QUICKSTEP
#include <parallel_quickstep/parallel_quickstep.h>
#endif

namespace gazebo
{
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

  /// \brief Add an entity to the world
  public: virtual void AddEntity(Entity *entity);

  /// \brief Remove an entity from the physics engine
  public: virtual void RemoveEntity(Entity *entity);

  /// \brief Create a new body
  public: virtual Body *CreateBody(Entity *parent);

  /// \brief Create a geom
  public: virtual Geom *CreateGeom(Shape::Type type, Body *parent);
 
  /// \brief Create a new joint
  public: virtual Joint *CreateJoint(Joint::Type type);

  /// \brief Return the space id 
  public: dSpaceID GetSpaceId() const;

  /// \brief Get the world id
  public: dWorldID GetWorldId();

  /// \brief Convert an odeMass to Mass
  public: virtual void ConvertMass(Mass *mass, void *odeMass);

  /// \brief Convert an odeMass to Mass
  public: virtual void ConvertMass(void *odeMass, const Mass &mass);

  /// \brief Get the step type
  public: virtual std::string GetStepType() const;

  /// \brief Set the step type
  public: virtual void SetStepType(const std::string type);

  /// \brief access functions to set ODE parameters
  public: void SetWorldCFM(double cfm);
  /// \brief access functions to set ODE parameters
  public: void SetWorldERP(double erp);
  /// \brief access functions to set ODE parameters
  public: void SetAutoDisableFlag(bool auto_disable);
  /// \brief access functions to set ODE parameters
  public: void SetSORPGSIters(unsigned int iters);
  /// \brief access functions to set ODE parameters
  public: void SetSORPGSW(double w);
  /// \brief access functions to set ODE parameters
  public: void SetContactMaxCorrectingVel(double vel);
  /// \brief access functions to set ODE parameters
  public: void SetContactSurfaceLayer(double layer_depth);
  /// \brief access functions to set ODE parameters
  public: void SetMaxContacts(int max_contacts);

#ifdef ODE_PRECON_PGS
  /// \brief access functions to set ODE parameters
  public: void SetSORPGSPreconIters(unsigned int iters);
  private: ParamT<unsigned int> *stepPreconItersP; 
  private: ParamT<int>    *quickStepPreconItersP;
#endif

  /// \brief access functions to set ODE parameters
  public: double GetWorldCFM();
  /// \brief access functions to set ODE parameters
  public: double GetWorldERP();
  /// \brief access functions to set ODE parameters
  public: bool GetAutoDisableFlag();
  /// \brief access functions to set ODE parameters
  public: int GetSORPGSIters();
  /// \brief access functions to set ODE parameters
  public: double GetSORPGSW();
  /// \brief access functions to set ODE parameters
  public: double GetContactMaxCorrectingVel();
  /// \brief access functions to set ODE parameters
  public: double GetContactSurfaceLayer();
  /// \brief access functions to set ODE parameters
  public: int GetMaxContacts();

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
  private: ParamT<std::string> *stepTypeP; 
  private: ParamT<unsigned int> *stepItersP; 
  private: ParamT<double> *stepWP; 
  private: ParamT<double> *contactMaxCorrectingVelP;
  private: ParamT<double> *contactSurfaceLayerP;
  private: ParamT<bool> *autoDisableBodyP;
  private: ParamT<int> *contactFeedbacksP;
  private: ParamT<int> *defaultMaxContactsP;

  /// \brief @todo: for backwards compatibility, should tick tock
  ///        deprecation as we switch to nested tags
  private: ParamT<bool>   *quickStepP;
  private: ParamT<int>    *quickStepItersP;
  private: ParamT<double> *quickStepWP;

  private: class ContactFeedback
           {
             public: Contact contact;
             public: std::vector<dJointFeedback> feedbacks;
           };

  private: std::vector<ContactFeedback> contactFeedbacks;
  private: std::vector<ContactFeedback>::iterator contactFeedbackIter;

  private: std::map<std::string, dSpaceID> spaces;

  private: std::vector<dContactGeom> contactGeoms;
  private: std::vector<bool> contactGeomsSkipped;

#ifdef QUICKSTEP_EXPERIMENTAL
  /// experimental ode stuff
  private: ParamT<int>    *islandThreadsP; // number of thread pool threads for islands
  private: ParamT<int>    *quickStepThreadsP; // number of thread pool threads for quickstep
  private: ParamT<int>    *quickStepChunksP; // number of thread pool threads for islands
  private: ParamT<int>    *quickStepOverlapP; // number of thread pool threads for islands
  private: ParamT<double> *quickStepToleranceP; // number of thread pool threads for islands
#endif

};

/** \}*/
/// \}
}

#endif
