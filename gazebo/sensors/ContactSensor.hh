/*
 * Copyright 2011 Nate Koenig
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

#include <stdint.h>
#include <vector>
#include <map>
#include <string>

#include "math/Angle.hh"
#include "sensors/Sensor.hh"
#include "physics/Contact.hh"

namespace gazebo
{
  namespace sensors
  {
    class Contact;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief Contact sensor.
    /// This sensor detects and reports collision contacts between objects.
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
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      public: virtual void Load(const std::string &_worldName);

      /// Initialize the sensor
      public: virtual void Init();

      ///  Update sensed values
      protected: virtual void UpdateImpl(bool force);

      /// Finalize the sensor
      protected: virtual void Fini();

      /// \brief Get the number of collisions that the sensor is observing
      public: unsigned int GetCollisionCount() const;

      /// \brief Get a collision name
      public: std::string GetCollisionName(unsigned int _index) const;

      /// \brief Return the number of contacts for an observed collision
      public: unsigned int GetCollisionContactCount(
                  const std::string &_collisionName) const;

      /// \brief Get a contact for a collision by index
      public: physics::Contact GetCollisionContact(
                  const std::string &_collisionName, unsigned int _index) const;

      /// \brief Returns a std::map of collision names and contacts
      ///   in collision with _collisionName.
      public: std::map<std::string, physics::Contact> GetContacts(
                  const std::string &_collisionName);

      private: void OnContact(const std::string &_collisionName,
                              const physics::Contact &_contact);

      private: std::vector<physics::CollisionPtr> collisions;

      private: typedef std::map<std::string,
               std::map<std::string, physics::Contact> > Contact_M;

      private: Contact_M contacts;

      private: transport::NodePtr node;
      private: transport::PublisherPtr contactsPub;

      private: boost::mutex *mutex;

      /// \brief returns a pointer to the mutex for locking while reading
      ///        internally kept map of map of collision names and contacts
      public: boost::mutex* GetUpdateMutex() const
              {return this->mutex;}
    };
    /// \}
  }
}

#endif


