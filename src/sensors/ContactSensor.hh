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
/* Desc: Contact sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
*/

#ifndef CONTACTSENSOR_HH
#define CONTACTSENSOR_HH

#include <vector>

#include <stdint.h>

#include "math/Angle.hh"
#include "sensors/Sensor.hh"
#include "physics/Contact.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{
   
    /// \brief Contact sensor.
    ///
    /// This sensor detects and reports contacts between objects 
    class ContactSensor: public Sensor
    {
      /// \brief Constructor
      /// \param body The underlying collision test uses an ODE collision, so
      ///             ray sensors must be attached to a body.
      public: ContactSensor();
    
      /// \brief Destructor
      public: virtual ~ContactSensor();
  
      /// Load the contact sensor using parameter from an XMLConfig node
      /// \param node The XMLConfig node
      protected: virtual void Load( sdf::ElementPtr &_sdf );
      protected: virtual void Load( );
    
      /// Initialize the sensor
      protected: virtual void InitChild();
    
      ///  Update sensed values
      protected: virtual void Update(bool force);
  
      /// Finalize the sensor
      protected: virtual void FiniChild();
  
      /// \brief Get the number of collisions that the sensor is observing
      public: unsigned int GetCollisionCount() const;
  
      /// \brief Get a collision
      public: physics::Collision *GetCollision(unsigned int index) const;
  
      /// \brief Return the number of contacts for an observed collision
      public: unsigned int GetCollisionContactCount(unsigned int collisionIndex) const;
  
      /// \brief Get a contact for a collision by index
      public: physics::Contact GetCollisionContact(unsigned int collision, unsigned int index) const;
  
      private: std::vector<physics::Collision *> collisions;

      private: gazebo::physics::WorldPtr world;
      private: gazebo::physics::ModelPtr model;
      private: gazebo::physics::LinkPtr link;
      private: gazebo::physics::CollisionPtr collision;

      public: gazebo::physics::ModelPtr GetParentModel() {return this->model;};

      public: std::vector<physics::Contact> GetContacts();
    };
    /// \}
  }
}

#endif
