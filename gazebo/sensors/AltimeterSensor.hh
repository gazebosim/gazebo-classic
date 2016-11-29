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

#ifndef _GAZEBO_SENSORS_ALTIMETERSENSOR_HH_
#define _GAZEBO_SENSORS_ALTIMETERSENSOR_HH_

#include <string>

#include <sdf/sdf.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class
    class AltimeterSensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief AltimeterSensor to provide vertical position and velocity
    class GAZEBO_VISIBLE AltimeterSensor: public Sensor
    {
      /// \brief Constructor.
      public: AltimeterSensor();

      /// \brief Destructor.
      public: virtual ~AltimeterSensor();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Accessor for current vertical position
      /// \return Current vertical position
      public: double Altitude() const;

      /// \brief Accessor for current vertical velocity
      /// \return Current vertical velocity
      public: double VerticalVelocity() const;

      /// \brief Accessor for the reference altitude
      /// \return Current reference altitude
      public: double ReferenceAltitude() const;

      /// \brief Accessor for current vertical velocity
      /// \param[in] _refAlt reference altitude
      public: void SetReferenceAltitude(const double _refAlt);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<AltimeterSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
