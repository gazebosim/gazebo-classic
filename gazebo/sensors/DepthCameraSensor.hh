/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_DEPTHCAMERASENSOR_HH_
#define _GAZEBO_SENSORS_DEPTHCAMERASENSOR_HH_

#include <memory>
#include <string>

#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Forward declare private data class
    class DepthCameraSensorPrivate;

    /// \class DepthCameraSensor DepthCameraSensor.hh sensors/sensors.hh
    /// \addtogroup gazebo_sensors Sensors
    /// \brief A set of sensor classes, functions, and definitions
    /// \{
    /// \brief Depth camera sensor
    /// This sensor is used for simulating standard monocular cameras
    class GAZEBO_VISIBLE DepthCameraSensor : public CameraSensor
    {
      /// \brief Constructor
      public: DepthCameraSensor();

      /// \brief Destructor
      public: virtual ~DepthCameraSensor();

      /// \brief Initialize the camera
      public: virtual void Init();

      /// \brief Gets the raw depth data from the sensor.
      /// \return The pointer to the depth data array.
      public: virtual const float *DepthData() const;

      /// \brief Returns a pointer to the rendering::DepthCamera
      /// \return Depth Camera pointer
      public: virtual rendering::DepthCameraPtr DepthCamera() const;

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      protected: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<DepthCameraSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
