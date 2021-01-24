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
#ifndef GAZEBO_SENSORS_CAMERASENSOR_PRIVATE_HH_
#define GAZEBO_SENSORS_CAMERASENSOR_PRIVATE_HH_

#include "gazebo/sensors/Sensor.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief CameraSensor private data
    class CameraSensorPrivate
    {
      /// \brief True if the sensor was rendered.
      public: bool rendered = false;

      /// \brief True if the sensor needs a rendering
      public: bool renderNeeded = false;

      /// \brief Rendering sensor extension class for ABI incompatible changes
      public: std::shared_ptr<RenderingSensorExt> extension;
    };
  }
}
#endif
