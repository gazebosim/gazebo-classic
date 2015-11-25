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
#ifndef _GAZEBO_CAMERASENSOR_HH_
#define _GAZEBO_CAMERASENSOR_HH_

#include <string>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class
    class CameraSensorPrivate;

    /// \addtogroup gazebo_sensors Sensors
    /// \{

    /// \class CameraSensor CameraSensor.hh sensors/sensors.hh
    /// \brief Basic camera sensor
    ///
    /// This sensor is used for simulating standard monocular cameras
    class GAZEBO_VISIBLE CameraSensor : public Sensor
    {
      /// \brief Constructor
      public: CameraSensor();

      /// \brief Destructor
      public: virtual ~CameraSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the camera
      public: virtual void Init();

      /// \brief Gets the topic name of the sensor
      /// \return Topic name
      public: virtual std::string Topic() const;

      /// \brief Returns a pointer to the rendering::Camera.
      /// \return The Pointer to the camera sensor.
      /// \deprecated See Camera() function
      public: rendering::CameraPtr GetCamera() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns a pointer to the rendering::Camera.
      /// \return The Pointer to the camera sensor.
      public: rendering::CameraPtr Camera() const;

      /// \brief Gets the width of the image in pixels.
      /// \return The image width in pixels.
      /// \deprecated See ImageWidth()
      public: unsigned int GetImageWidth() const GAZEBO_DEPRECATED(7.0);

      /// \brief Gets the width of the image in pixels.
      /// \return The image width in pixels.
      /// \deprecated See ImageWidth()
      public: unsigned int ImageWidth() const;

      /// \brief Gets the height of the image in pixels.
      /// \return The image height in pixels.
      /// \deprecated See ImageHeight()
      public: unsigned int GetImageHeight() const GAZEBO_DEPRECATED(7.0);

      /// \brief Gets the height of the image in pixels.
      /// \return The image height in pixels.
      public: unsigned int ImageHeight() const;

      /// \brief Gets the raw image data from the sensor.
      /// \return The pointer to the image data array.
      /// \deprecated See ImageData()
      public: const unsigned char *GetImageData() GAZEBO_DEPRECATED(7.0);

      /// \brief Gets the raw image data from the sensor.
      /// \return The pointer to the image data array.
      public: const unsigned char *ImageData() const;

      /// \brief Saves the image to the disk.
      /// \param[in] _filename The name of the file to be saved.
      /// \return True if successful, false if unsuccessful.
      public: bool SaveFrame(const std::string &_filename);

      // Documentation inherited
      public: virtual bool IsActive() const;

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to private data.
      protected: CameraSensor(CameraSensorPrivate &_dataPtr);

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      /// \brief Finalize the camera
      protected: virtual void Fini();

      /// \brief Handle the render event.
      private: void Render();

      /// \internal
      /// \brief Private data pointer
      private: std::shared_ptr<CameraSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
