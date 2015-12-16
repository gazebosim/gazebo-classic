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
#ifndef _GAZEBO_SENSORS_NOISE_PRIVATE_HH_
#define _GAZEBO_SENSORS_NOISE_PRIVATE_HH_

#include <functional>
#include <sdf/sdf.hh>

#include "gazebo/sensors/SensorTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Private data for Noise class
    class NoisePrivate
    {
      public: NoisePrivate(const NoiseType _type) : type(_type) {}

      /// \brief Which type of noise we're applying
      public: NoiseType type;

      /// \brief Noise sdf element.
      public: sdf::ElementPtr sdf;

      /// \brief Callback function for applying custom noise to sensor data.
      public: std::function<double (double)> customNoiseCallback;
    };
  }
}
#endif
