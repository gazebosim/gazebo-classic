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

#ifndef _SONARSENSOR_HH_
#define _SONARSENSOR_HH_

#include <vector>
#include <string>

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class SonarSensor SonarSensor.hh sensors/sensors.hh
    /// \brief Sensor with sonar cone.
    ///
    /// This sensor uses a cone .
    class SonarSensor: public Sensor
    {
      /// \brief Constructor
      public: SonarSensor();

      /// \brief Destructor
      public: virtual ~SonarSensor();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      /// \brief Get detected range for a sonar.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \return Returns DBL_MAX for no detection.
      public: double GetRange();

      // Documentation inherited
      public: virtual bool IsActive();

      /// \brief Callback for contact messages from the physics engine.
      private: void OnContacts(ConstContactsPtr &_msg);

      private: physics::CollisionPtr sonarCollision;
      private: physics::CylinderShapePtr sonarShape;
      private: physics::EntityPtr parentEntity;

      /// \brief Subscription to contact messages from the physics engine
      private: transport::SubscriberPtr contactSub;

      private: transport::PublisherPtr sonarPub;
      private: msgs::SonarStamped sonarMsg;

      private: boost::mutex mutex;

      typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;
      private: ContactMsgs_L incomingContacts;

      private: math::Pose sonarMidPose;
    };
    /// \}
  }
}
#endif
