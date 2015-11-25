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
#ifndef _GAZEBO_DEPTHCAMERASENSOR_HH_
#define _GAZEBO_DEPTHCAMERASENSOR_HH_

#include <string>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
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
    class GAZEBO_VISIBLE DepthCameraSensor : public Sensor
    {
      /// \brief Constructor
      public: DepthCameraSensor();

      /// \brief Destructor
      public: virtual ~DepthCameraSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      protected: virtual void Load(const std::string &_worldName,
                                   sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      protected: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the camera
      protected: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      /// Finalize the camera
      protected: virtual void Fini();

      /// \brief Set whether the sensor is active or not
      /// \param[in] _value True if active, false if not
      public: virtual void SetActive(const bool _value);

      /// \brief Returns a pointer to the rendering::DepthCamera
      /// \return Depth Camera pointer
      /// \deprecated See DepthCamera()
      public: rendering::DepthCameraPtr GetDepthCamera() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Returns a pointer to the rendering::DepthCamera
      /// \return Depth Camera pointer
      /// \deprecated See DepthCamera()
      public: rendering::DepthCameraPtr DepthCamera() const;

      /// \brief Saves an image frame of depth camera sensor to file
      /// \param[in] Name of file to save as
      /// \return True if saved, false if not
      public: bool SaveFrame(const std::string &_filename);

      /// \brief Handle the render event.
      private: void Render();

      /// \internal
      /// \brief Private data pointer
      private: std::shared_ptr<DepthCameraSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
