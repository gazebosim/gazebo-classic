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
/* Desc: The base class for all physics engines
 * Author: Nate Koenig
 */

#ifndef PHYSICSENGINE_HH
#define PHYSICSENGINE_HH

#include "common/Event.hh"
#include "common/CommonTypes.hh"
#include "msgs/msgs.h"
#include "transport/TransportTypes.hh"
#include "physics/PhysicsTypes.hh"
#include <boost/thread/recursive_mutex.hpp>

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
  
    /// \brief Base class for a physics engine
    class PhysicsEngine
    {
      /// \brief Default constructor
      /// \param world Pointer to the world
      public: PhysicsEngine(WorldPtr world);
    
      /// \brief Destructor
      public: virtual ~PhysicsEngine();
    
      /// \brief Load the physics engine
      /// \param _sdf Pointer to the SDF parameters
      public: virtual void Load( sdf::ElementPtr _sdf ) = 0;
    
      /// \brief Initialize the physics engine
      public: virtual void Init() = 0;

      /// \brief Finilize the physics engine
      public: virtual void Fini();
   
      /// \brief Init the engine for threads. 
      public: virtual void InitForThread() = 0;
  
      /// \brief Update the physics engine collision
      public: virtual void UpdateCollision() = 0;

      /// \brief Get the simulation step time
      public: virtual double GetStepTime() = 0;
  
      /// \brief Update the physics engine
      public: virtual void UpdatePhysics() {}
    
      /// \brief Create a new body
      public: virtual LinkPtr CreateLink(ModelPtr _parent) = 0;
  
      /// \brief Create a collision
      public: virtual CollisionPtr CreateCollision(const std::string &shapeTypename, 
                                         LinkPtr body) = 0;
  
      /// \brief Create a new joint
      public: virtual JointPtr CreateJoint(const std::string &type) = 0;
    
      /// \brief Return the gavity vector
      /// \return The gavity vector
      public: math::Vector3 GetGravity() const;
  
      /// \brief Set the gavity vector
      public: virtual void SetGravity(const gazebo::math::Vector3 &gravity) = 0;
  
      /// \brief Set whether to show contacts
      public: void ShowContacts(const bool &show);
  
      /// \brief access functions to set ODE parameters
      public: virtual void SetWorldCFM(double /*cfm_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetWorldERP(double /*erp_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetAutoDisableFlag(bool /*autoDisable_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetSORPGSIters(unsigned int /*iters_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetSORPGSW(double /*w_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetContactMaxCorrectingVel(double /*vel_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetContactSurfaceLayer(double /*layerDepth_*/) {}
      /// \brief access functions to set ODE parameters
      public: virtual void SetMaxContacts(double /*maxContacts_*/) {}
  
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
  
      /// \brief Add a contact visual
      protected: void AddContactVisual(const math::Vector3 &pos_, 
                                       const math::Vector3 &norm_);
      protected: virtual void OnRequest( 
                 const boost::shared_ptr<msgs::Request const> &/*_msg*/ )
                 {}

      protected: virtual void OnPhysicsMsg( 
                 const boost::shared_ptr<msgs::Physics const> &/*_msg*/ )
                 {}



      protected: WorldPtr world;
      protected: sdf::ElementPtr sdf;
  
      protected: std::string visual;
      protected: transport::NodePtr node;
      protected: transport::PublisherPtr visPub;
      protected: transport::PublisherPtr responsePub;
      protected: transport::SubscriberPtr physicsSub, requestSub;
      private: event::ConnectionPtr showContactConnection; 
  
      //private: std::vector<OgreDynamicLines*> contactLines;
      //private: std::vector<OgreDynamicLines*>::iterator contactLinesIter;
    };
    
    /// \}
  }

}
#endif
