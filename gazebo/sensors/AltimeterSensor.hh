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

#ifndef _ALTIMETER_SENSOR_HH_
#define _ALTIMETER_SENSOR_HH_

#include <string>

#include <sdf/sdf.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class AltimeterSensor AltimeterSensor.hh sensors/sensors.hh
    /// \brief AltimeterSensor to provide vertical position and velocity
    class GAZEBO_VISIBLE AltimeterSensor: public Sensor
    {
      /// \brief Constructor.
      public: AltimeterSensor();

      /// \brief Destructor.
      public: virtual ~AltimeterSensor();

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Accessor for current vertical position
      /// \return Current vertical position
      public: double GetVerticalPosition();

      /// \brief Accessor for current vertical velocity
      /// \return Current vertical velocity
      public: double GetVerticalVelocity();

      /// \brief Accessor for the reference altitude
      /// \return Current reference altitude
      public: double GetReferenceAltitude();

      /// \brief Accessor for current vertical velocity
      /// \param[in] _refAlt reference altitude
      public: void SetReferenceAltitude(double _refAlt);

      /// \brief Mutex to protect reads and writes.
      private: mutable boost::mutex mutex;
      
      /// \brief GPS data publisher.
      private: transport::PublisherPtr altPub;

      /// \brief Topic name for GPS data publisher.
      private: std::string topicName;

      /// \brief Parent link of this sensor.
      private: physics::LinkPtr parentLink;

      /// \brief Stores most recent altimeter sensor data.
      private: msgs::Altimeter altMsg;
    };
    /// \}
  }
}
#endif
