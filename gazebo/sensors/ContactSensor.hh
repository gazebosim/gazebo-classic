/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _CONTACTSENSOR_HH_
#define _CONTACTSENSOR_HH_

#include <vector>
#include <map>
#include <list>
#include <string>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/physics/Contact.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class ContactSensor ContactSensor.hh sensors/sensors.hh
    /// \brief Contact sensor. This sensor detects and reports contacts between
    ///  objects
    class ContactSensor: public Sensor
    {
      /// \brief Constructor.
      public: ContactSensor();

      /// \brief Destructor.
      public: virtual ~ContactSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters.
      /// \param[in] _worldName Name of world to load from.
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the sensor.
      public: virtual void Init();

      /// \brief Update the sensor information.
      /// \param[in] _force True if update is forced, false if not.
      protected: virtual void UpdateImpl(bool _force);

      /// \brief Finalize the sensor.
      protected: virtual void Fini();

      /// \brief Get the number of collisions that the sensor is observing.
      /// \return Number of collisions.
      public: unsigned int GetCollisionCount() const;

      /// \brief Get a collision name at index _index.
      /// \param[in] _index Index of collision in collection of collisions.
      /// \return name of collision.
      public: std::string GetCollisionName(unsigned int _index) const;

      /// \brief Return the number of contacts for an observed collision.
      /// \param[in] _collisionName The name of the observed collision.
      /// \return The collision contact count.
      public: unsigned int GetCollisionContactCount(
                  const std::string &_collisionName) const;


      /// \brief Get all the contacts
      /// \return Message that contains all the contact information
      public: msgs::Contacts GetContacts() const;

      /// \brief Gets contacts of a collision
      /// \param[in] _collisionName Name of collision
      /// \return Container of contacts
      public: std::map<std::string, physics::Contact> GetContacts(
                  const std::string &_collisionName);

      // Documentation inherited.
      public: virtual bool IsActive();

      /// \brief Callback for contact messages from the physics engine.
      private: void OnContacts(ConstContactsPtr &_msg);

      /// \brief Collisions this sensor monitors for contacts
      private: std::vector<std::string> collisions;

      /// \brief Output contact information.
      private: transport::PublisherPtr contactsPub;

      /// \brief Subscription to contact messages from the physics engine
      private: transport::SubscriberPtr contactSub;

      /// \brief Mutex to protect reads and writes.
      private: boost::mutex mutex;

      /// \brief Contacts message used to output sensor data.
      private: msgs::Contacts contactsMsg;

      typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;
      private: ContactMsgs_L incomingContacts;
    };
    /// \}
  }
}
#endif
