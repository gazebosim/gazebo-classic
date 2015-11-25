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
#ifndef _GAZEBO_GPSSENSOR_HH_
#define _GAZEBO_GPSSENSOR_HH_

#include <string>

#include <sdf/sdf.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class
    class GpsSensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class GpsSensor GpsSensor.hh sensors/sensors.hh
    /// \brief GpsSensor to provide position measurement.
    class GAZEBO_VISIBLE GpsSensor: public Sensor
    {
      /// \brief Constructor.
      public: GpsSensor();

      /// \brief Destructor.
      public: virtual ~GpsSensor();

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Accessor for current longitude angle
      /// \return Current longitude angle.
      public: ignition::math::Angle Longitude() const;

      /// \brief Accessor for current latitude angle
      /// \return Current latitude angle.
      public: ignition::math::Angle Latitude() const;

      /// \brief Accessor for current altitude
      /// \return Current altitude above sea level.
      /// \deprecated See Altitude()
      public: double GetAltitude() const GAZEBO_DEPRECATED(7.0);

      /// \brief Accessor for current altitude
      /// \return Current altitude above sea level.
      public: double Altitude() const;

      /// \internal
      /// \brief Private data pointer
      private: std::shared_ptr<GpsSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
