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
#ifndef _GAZEBO_SENSORS_MAGNETOMETER_SENSOR_HH_
#define _GAZEBO_SENSORS_MAGNETOMETER_SENSOR_HH_

#include <string>

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data
    class MagnetometerSensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief MagnetometerSensor to provide magnetic field measurement.
    class GAZEBO_VISIBLE MagnetometerSensor: public Sensor
    {
      /// \brief Constructor.
      public: MagnetometerSensor();

      /// \brief Destructor.
      public: virtual ~MagnetometerSensor();

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Accessor for current magnetic field in Tesla
      /// \return Current magnetic field
      public: ignition::math::Vector3d MagneticField() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<MagnetometerSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
