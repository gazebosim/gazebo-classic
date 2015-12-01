/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_SONARSENSOR_HH_
#define _GAZEBO_SENSORS_SONARSENSOR_HH_

#include <string>
#include <list>
#include <ignition/math/Pose3.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Forward declare private data class.
    class SonarSensorPrivate;

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
      public: virtual std::string Topic() const;

      /// \brief Get the minimum range of the sonar
      /// \return The sonar's minimum range.
      /// \deprecated See RangeMin()
      public: double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the minimum range of the sonar
      /// \return The sonar's minimum range.
      public: double RangeMin() const;

      /// \brief Get the minimum range of the sonar.
      /// \return The sonar's maximum range.
      /// \deprecated See RangeMax()
      public: double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the minimum range of the sonar.
      /// \return The sonar's maximum range.
      public: double RangeMax() const;

      /// \brief Get the radius of the sonar cone at maximum range.
      /// \return The radisu of the sonar cone at max range.
      /// \deprecated See Radius()
      public: double GetRadius() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the radius of the sonar cone at maximum range.
      /// \return The radisu of the sonar cone at max range.
      /// \deprecated See Radius()
      public: double Radius() const;

      /// \brief Get detected range for a sonar.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \return Returns DBL_MAX for no detection.
      /// \deprecated See Range()
      public: double GetRange() GAZEBO_DEPRECATED(7.0);

      /// \brief Get detected range for a sonar.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \return Returns DBL_MAX for no detection.
      public: double Range();


      // Documentation inherited
      public: virtual bool IsActive();

      /// \brief Connect a to the new update signal.
      /// \param[in] _subscriber Callback function.
      /// \return The connection, which must be kept in scope.
      public: event::ConnectionPtr ConnectUpdate(
                  std::function<void (msgs::SonarStamped)> _subscriber);

      /// \brief Disconnect from the update signal.
      /// \param[in] _conn Connection to remove.
      public: void DisconnectUpdate(event::ConnectionPtr &_conn);

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Callback for contact messages from the physics engine.
      private: void OnContacts(ConstContactsPtr &_msg);

      /// \internal
      /// \brief Internal data pointer
      private: std::shared_ptr<SonarSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
