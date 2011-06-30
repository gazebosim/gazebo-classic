/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: The ODE physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */

#ifndef ODEPHYSICS_HH
#define ODEPHYSICS_HH

#include "physics/ode/ode_inc.h"

#include <tbb/spin_mutex.h>
#include <tbb/concurrent_vector.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "physics/ode/ODETypes.hh"

#include "physics/PhysicsEngine.hh"
#include "physics/Contact.hh"
#include "physics/Shape.hh"
#include "gazebo_config.h"

namespace gazebo
{
	namespace physics
  {
    class ContactFeedback
    {
      public: Contact contact;
      public: std::vector<dJointFeedback> feedbacks;
    };

  /// \brief ODE physics engine
  class ODEPhysics : public PhysicsEngine
  {
    /// \brief Constructor
    public: ODEPhysics(WorldPtr world);
  
    /// \brief Destructor
    public: virtual ~ODEPhysics();
  
    /// \brief Load the ODE engine
    public: virtual void Load( sdf::ElementPtr _sdf );
  
    /// \brief Initialize the ODE engine
    public: virtual void Init();
  
    /// \brief Init the engine for threads. 
    public: virtual void InitForThread();
  
    /// \brief Update the ODE collision
    public: virtual void UpdateCollision();
  
    /// \brief Update the ODE engine
    public: virtual void UpdatePhysics();
  
    /// \brief Finilize the ODE engine
    public: virtual void Fini();

    /// \brief Get the simulation step time
    public: virtual double GetStepTime();
 
    /// \brief Create a new body
    public: virtual BodyPtr CreateBody(EntityPtr parent);
  
    /// \brief Create a geom
    public: virtual GeomPtr CreateGeom(const std::string &shapeTypename, 
                                       BodyPtr parent);
   
    /// \brief Create a new joint
    public: virtual JointPtr CreateJoint(const std::string &type);
  
    /// \brief Return the space id 
    public: dSpaceID GetSpaceId() const;
  
    /// \brief Get the world id
    public: dWorldID GetWorldId();
  
    /// \brief Convert an odeMass to Mass
    public: static void ConvertMass(Mass *mass, void *odeMass);
  
    /// \brief Convert an odeMass to Mass
    public: static void ConvertMass(void *odeMass, const Mass &mass);
  
    /// \brief Get the step type
    public: virtual std::string GetStepType() const;
  
    /// \brief Set the step type
    public: virtual void SetStepType(const std::string type);
  
    /// \brief Set the gavity vector
    public: virtual void SetGravity(const gazebo::math::Vector3 &gravity);
  
    /// \brief access functions to set ODE parameters
    public: void SetWorldCFM(double cfm);
    /// \brief access functions to set ODE parameters
    public: void SetWorldERP(double erp);
    /// \brief access functions to set ODE parameters
    public: void SetSORPGSIters(unsigned int iters);
    /// \brief access functions to set ODE parameters
    public: void SetSORPGSW(double w);
    /// \brief access functions to set ODE parameters
    public: void SetContactMaxCorrectingVel(double vel);
    /// \brief access functions to set ODE parameters
    public: void SetContactSurfaceLayer(double layer_depth);
    /// \brief access functions to set ODE parameters
    public: void SetMaxContacts(unsigned int max_contacts);
  
    /// \brief access functions to set ODE parameters
    public: double GetWorldCFM();
    /// \brief access functions to set ODE parameters
    public: double GetWorldERP();
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
  
    public: void CreateContact(ODEGeom *geom1, ODEGeom *geom2);
  
    /// \brief Do collision detection
    private: static void CollisionCallback( void *data, dGeomID o1, dGeomID o2);
  
    /// \brief Collide two geoms
    public: void Collide(ODEGeom *geom1, ODEGeom *geom2, 
                         dContactGeom *contactGeoms);
  
    public: void ProcessContactFeedback(ContactFeedback &feedback);

    /// \brief Top-level world for all bodies
    private: dWorldID worldId;
  
    /// \brief Top-level space for all sub-spaces/geoms
    private: dSpaceID spaceId;
  
    /// \brief Collision attributes
    private: dJointGroupID contactGroup;
  
    /// Store the value of the stepTime parameter in doubl form. To improve
    /// efficiency
    private: double stepTimeDouble; 
    private: std::string stepType;

    private: tbb::concurrent_vector<ContactFeedback> contactFeedbacks;
  
    private: std::map<std::string, dSpaceID> spaces;

    private: std::vector< std::pair<ODEGeom*, ODEGeom*> > colliders;
    private: std::vector< std::pair<ODEGeom*, ODEGeom*> > trimeshColliders;
  
    private: tbb::spin_mutex collideMutex;

    private: dContactGeom *contactGeoms;
#if ODE_WG_TRUNK
    private: int (*physicsStepFunc)(dxWorld*, dReal);
#else
    private: void (*physicsStepFunc)(dxWorld*, dReal);
#endif
  };
  
  
  /** \}*/
  /// \}
  }
}
#endif
