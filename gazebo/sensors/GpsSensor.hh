/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _GPSSENSOR_HH_
#define _GPSSENSOR_HH_

#include <string>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sdf/sdf.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class GpsSensor GpsSensor.hh sensors/sensors.hh
    /// \brief GpsSensor to provide position measurement.
    class GpsSensor: public Sensor
    {
      /// \brief Constructor.
      public: GpsSensor();

      /// \brief Destructor.
      public: virtual ~GpsSensor();

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr &_sdf);

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Parent entity of this sensor.
      private: physics::EntityPtr parentEntity;

      /// \brief Pointer to SphericalCoordinates converter.
      private: common::SphericalCoordinatesPtr sphericalCoordinates;
    };
    /// \}
  }
}
#endif
