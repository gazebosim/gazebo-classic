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
#include "transport/TransportTypes.hh"
#include "physics/PhysicsTypes.hh"

namespace gazebo
{
	namespace physics
  {
   
    /// \brief Base class for a physics engine
    class PhysicsEngine
    {
      /// \brief Default constructor
      /// \param world Pointer to the world
      public: PhysicsEngine(WorldPtr world);
    
      /// \brief Destructor
      public: virtual ~PhysicsEngine();
    
      /// \brief Load the physics engine
      /// \param node Pointer to the XML parameters
      public: virtual void Load(common::XMLConfigNode *node) = 0;
    
      /// \brief Saves to XMLFile
      /// \param stread Output stream
      public: virtual void Save(std::string &prefix, std::ostream &stream) =0;
    
      /// \brief Initialize the physics engine
      public: virtual void Init() = 0;
   
      /// \brief Init the engine for threads. 
      public: virtual void InitForThread() = 0;
  
      /// \brief Update the physics engine collision
      public: virtual void UpdateCollision() = 0;
  
      /// \brief Update the physics engine
      public: virtual void UpdatePhysics() {}
    
      /// \brief Finilize the physics engine
      public: virtual void Fini() = 0;
  
      /// \brief Create a new body
      public: virtual BodyPtr CreateBody(EntityPtr parent) = 0;
  
      /// \brief Create a geom
      public: virtual GeomPtr CreateGeom(const std::string &shapeTypename, 
                                         BodyPtr body) = 0;
  
      /// \brief Create a new joint
      public: virtual JointPtr CreateJoint(const std::string &type) = 0;
    
      /// \brief Return the gavity vector
      /// \return The gavity vector
      public: common::Vector3 GetGravity() const;
  
      /// \brief Set the gavity vector
      public: virtual void SetGravity(const gazebo::common::Vector3 &gravity) = 0;
  
      /// \brief Get the time between each update cycle
      /// \return seconds between updates 
      public: double GetUpdateRate() const;
  
      /// \brief Set the time between each update cycle
      public: void SetUpdateRate(double rate) const;
  
      /// \brief Get the physics time steps in the virtual world
      /// \return step time 
      public: common::Time GetStepTime() const;
  
      /// \brief Set the step time
      public: void SetStepTime(common::Time time);
  
      /// \brief Get the step type
      public: virtual std::string GetStepType() const {return "unknown";}
  
      /// \brief Set the step type
      public: virtual void SetStepType(const std::string type) {}
  
      /// \brief Set whether to show contacts
      public: void ShowContacts(const bool &show);
  
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
      public: virtual void SetMaxContacts(double max_contacts) {}
  
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
  
      /// \brief Get the count of the parameters
      public: unsigned int GetParamCount() const;
  
      /// \brief Get a param by index
      public: common::Param *GetParam(unsigned int index) const;
  
      /// \brief Get a parameter by name
      public: common::Param *GetParam(const std::string &key) const;
  
       /// \brief Set a parameter by name
      public: void SetParam(const std::string &key, const std::string &value);
   
      /// \brief Add a contact visual
      protected: void AddContactVisual(common::Vector3 pos, common::Vector3 norm);
  
      protected: WorldPtr world;
  
      /// The gravity vector
      protected: common::ParamT<common::Vector3> *gravityP;
    
      /// time steps the physical engine will take 
      /// how much time will pass on each update
      protected: common::ParamT<common::Time> *stepTimeP;
      
      /// update rate of the physical engine, how many times
      /// it is called 
      protected: common::ParamT<double> *updateRateP;
  
      protected: common::Param_V parameters;
  
      protected: std::string visual;
      protected: transport::NodePtr node;
      protected: transport::PublisherPtr vis_pub;
      private: event::ConnectionPtr showContactConnection; 
  
      //private: std::vector<OgreDynamicLines*> contactLines;
      //private: std::vector<OgreDynamicLines*>::iterator contactLinesIter;
    };
    
    /** \}*/
  }

}
#endif
