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

#include <iostream>

#include "Joint.hh"
#include "Param.hh"
#include "Geom.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  
  class Entity;
  class Body;
  class XMLConfigNode;
  class OgreVisual;
  
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
  
    /// \brief Saves to XMLFile
    /// \param stread Output stream
    public: virtual void Save(std::string &prefix, std::ostream &stream) =0;
  
    /// \brief Initialize the physics engine
    public: virtual void Init() = 0;

    /// \brief Initialize for separate thread
    public: virtual void InitForThread() = 0;
  
    /// \brief Update the physics engine collision
    public: virtual void UpdateCollision() = 0;

    /// \brief Update the physics engine
    public: virtual void UpdatePhysics();
  
    /// \brief Finilize the physics engine
    public: virtual void Fini() = 0;

    /// \brief Add an entity to the world
    public: virtual void AddEntity(Entity *entity) = 0;

    /// \brief Remove an entity from the physics engine
    public: virtual void RemoveEntity(Entity *entity) = 0;
  
    /// \brief Create a new body
    public: virtual Body *CreateBody(Entity *parent) = 0;

    /// \brief Create a geom
    public: virtual Geom *CreateGeom(Shape::Type type, Body *body) = 0;

    /// \brief Create a geom
    public: Geom *CreateGeom(std::string typeName, Body *body);
  
    /// \brief Create a new joint
    public: virtual Joint *CreateJoint(Joint::Type type) = 0;
  
    /// \brief stores rms errors from quickstep
    public: virtual double GetRMSError() = 0;

    /// \brief Return the gavity vector
    /// \return The gavity vector
    public: Vector3 GetGravity() const;

    /// \brief Set the gavity vector
    public: void SetGravity(Vector3 gravity) const;

    /// \brief Get the time between each update cycle
    /// \return seconds between updates 
    public: double GetUpdateRate() const;

    /// \brief Set the time between each update cycle
    public: void SetUpdateRate(double rate) const;

    /// \brief Get the physics time steps in the virtual world
    /// \return step time 
    public: Time GetStepTime() const;

    /// \brief Set the step time
    public: void SetStepTime(Time time);

    /// \brief Get the step type
    public: virtual std::string GetStepType() const {return "unknown";}

    /// \brief Set the step type
    public: virtual void SetStepType(const std::string type) {}

    /// \brief Lock the physics engine mutex
    public: void LockMutex();

    /// \brief Unlock the physics engine mutex
    public: void UnlockMutex();

    /// \brief Convert an engine specific mass to a gazeboMass
    public: virtual void ConvertMass(Mass *mass, void *engineMass) = 0;

    /// \brief Convert a Gazebo mass to an engine specific mass
    public: virtual void ConvertMass(void *engineMass, const Mass &mass) = 0;

    /// \brief Set whether to show contacts
    public: void ShowVisual(bool show);

    /// \brief access functions to set ODE parameters
    public: virtual void SetWorldCFM(double cfm) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetWorldERP(double erp) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetAutoDisableFlag(bool auto_disable) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetSORPGSIters(unsigned int iters) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetSORPGSW(double w) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetContactMaxCorrectingVel(double vel) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetContactSurfaceLayer(double layer_depth) {}
    /// \brief access functions to set ODE parameters
    public: virtual void SetMaxContacts(int max_contacts) {}

    /// \brief access functions to set ODE parameters
    public: virtual double GetWorldCFM() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual double GetWorldERP() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual bool GetAutoDisableFlag() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual int GetSORPGSIters() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual double GetSORPGSW() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual double GetContactMaxCorrectingVel() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual double GetContactSurfaceLayer() {return 0;}
    /// \brief access functions to set ODE parameters
    public: virtual int GetMaxContacts() {return 0;}

    /// \brief Setup contact visuals
    public: void SetContactVisual(int num_contacts);

    /// \brief Add a contact visual
    protected: void AddContactVisual(Vector3 pos, Vector3 norm);

    /// The gravity vector
    protected: ParamT<Vector3> *gravityP;
  
    /// time steps the physical engine will take 
    /// how much time will pass on each update
    protected: ParamT<Time> *stepTimeP;
    
    /// update rate of the physical engine, how many times
    /// it is called 
    protected: ParamT<double> *updateRateP;

    protected: std::vector<Param*> parameters;

    private: boost::recursive_mutex *mutex;

    protected: OgreVisual *visual;

    private: std::vector<OgreDynamicLines*> contactLines;
    private: std::vector<OgreDynamicLines*>::iterator contactLinesIter;
  };
  
  /** \}*/
}

#endif
