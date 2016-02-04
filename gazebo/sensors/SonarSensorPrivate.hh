/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_SONARSENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_SONARSENSOR_PRIVATE_HH_

#include <list>
#include <mutex>
#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Sonar sensor private data
    class SonarSensorPrivate
    {
      /// \brief Contact messages list type
      typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

      /// \brief Update event.
      public: event::EventT<void(msgs::SonarStamped)> update;

      /// \brief Collison object that holds the sonarShape.
      public: physics::CollisionPtr sonarCollision;

      /// \brief Shape used to generate contact information.
      public: physics::MeshShapePtr sonarShape;

      /// \brief Parent entity of this sensor
      public: physics::EntityPtr parentEntity;

      /// \brief Subscription to contact messages from the physics engine
      public: transport::SubscriberPtr contactSub;

      /// \brief Publishes the sonarMsg.
      public: transport::PublisherPtr sonarPub;

      /// \brief Store current sonar info
      public: msgs::SonarStamped sonarMsg;

      /// \brief Mutex used to protect reading/writing the sonar message.
      public: std::mutex mutex;

      /// \brief List of received contacts from the sonar collision shape.
      public: ContactMsgs_L incomingContacts;

      /// \brief Pose of the sonar shape's midpoint.
      public: ignition::math::Pose3d sonarMidPose;

      /// \brief Minimum range
      public: double rangeMin;

      /// \brief Maximum range
      public: double rangeMax;

      /// \brief Radius of the sonar cone at maximum range.
      public: double radius;

      /// \brief Counts the number of times there were no contacts. This is
      /// used to reduce the range value jumping.
      public: int emptyContactCount;
    };
  }
}
#endif
