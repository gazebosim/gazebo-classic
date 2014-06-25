/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <string>
#include <list>

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

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
    class GAZEBO_VISIBLE SonarSensor: public Sensor
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
      protected: virtual bool UpdateImpl(bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      /// \brief Get the minimum range of the sonar
      /// \return The sonar's minimum range.
      public: double GetRangeMin() const;

      /// \brief Get the minimum range of the sonar.
      /// \return The sonar's maximum range.
      public: double GetRangeMax() const;

      /// \brief Get the radius of the sonar cone at maximum range.
      /// \return The radisu of the sonar cone at max range.
      public: double GetRadius() const;

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

      /// \brief Connect a to the new update signal.
      /// \param[in] _subscriber Callback function.
      /// \return The connection, which must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectUpdate(T _subscriber)
              {return update.Connect(_subscriber);}

      /// \brief Disconnect from the update signal.
      /// \param[in] _conn Connection to remove.
      public: void DisconnectUpdate(event::ConnectionPtr &_conn)
              {update.Disconnect(_conn);}

      /// \brief Callback for contact messages from the physics engine.
      private: void OnContacts(ConstContactsPtr &_msg);

      /// \brief Collison object that holds the sonarShape.
      private: physics::CollisionPtr sonarCollision;

      /// \brief Shape used to generate contact information.
      private: physics::MeshShapePtr sonarShape;

      /// \brief Parent entity of this sensor
      private: physics::EntityPtr parentEntity;

      /// \brief Subscription to contact messages from the physics engine
      private: transport::SubscriberPtr contactSub;

      /// \brief Publishes the sonsarMsg.
      private: transport::PublisherPtr sonarPub;

      /// \brief Store current sonar info
      private: msgs::SonarStamped sonarMsg;

      /// \brief Mutex used to protect reading/writing the sonar message.
      private: boost::mutex mutex;

      /// \brief Contact messages list type
      typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

      /// \brief List of received contacts from the sonar collision shape.
      private: ContactMsgs_L incomingContacts;

      /// \brief Pose of the sonar shape's midpoint.
      private: math::Pose sonarMidPose;

      /// \brief Minimum range
      private: double rangeMin;

      /// \brief Maximum range
      private: double rangeMax;

      /// \brief Radius of the sonar cone at maximum range.
      private: double radius;

      /// \brief Update event.
      protected: event::EventT<void(msgs::SonarStamped)> update;
    };
    /// \}
  }
}
#endif
