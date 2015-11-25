/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_CONTACTSENSOR_PRIVATE_HH_
#define _GAZEBO_CONTACTSENSOR_PRIVATE_HH_

#include <vector>
#include <list>
#include <string>
#include <mutex>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorPrivate.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Contact sensor private data.
    class ContactSensorPrivate : public SensorProtected
    {
      /// \brief Collisions this sensor monitors for contacts
      public: std::vector<std::string> collisions;

      /// \brief Output contact information.
      public: transport::PublisherPtr contactsPub;

      /// \brief Subscription to contact messages from the physics engine
      public: transport::SubscriberPtr contactSub;

      /// \brief Mutex to protect reads and writes.
      public: mutable std::mutex mutex;

      /// \brief Contacts message used to output sensor data.
      public: msgs::Contacts contactsMsg;

      /// \type ContactMsgs_L
      /// List of contact messages
      typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

      /// \brief List of incoming messages.
      public: ContactMsgs_L incomingContacts;
    };
  }
}
#endif
