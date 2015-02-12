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

#ifndef _MULTICAMERASENSOR_HH_
#define _MULTICAMERASENSOR_HH_

#include <string>
#include <vector>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors Sensors
    /// \brief A set of sensor classes, functions, and definitions
    /// \{

    /// \class MultiCameraSensor MultiCameraSensor.hh sensors/sensors.hh
    /// \brief Multiple camera sensor. This sensor type can create one or
    /// more synchronized cameras.
    class GAZEBO_VISIBLE MultiCameraSensor : public Sensor
    {
      /// \brief Constructor
      public: MultiCameraSensor();

      /// \brief Destructor
      public: virtual ~MultiCameraSensor();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      /// \brief Get the number of cameras.
      /// \return The number of cameras.
      public: unsigned int GetCameraCount() const;

      /// \brief Returns a pointer to a rendering::Camera.
      /// \param[in] _index Index of the camera to get
      /// \return The Pointer to the camera sensor.
      /// \sa MultiCameraSensor::GetCameraCount
      public: rendering::CameraPtr GetCamera(unsigned int _index) const;

      /// \brief Gets the width of the image in pixels.
      /// \param[in] _index Index of the camera
      /// \return The image width in pixels.
      /// \sa MultiCameraSensor::GetCameraCount
      public: unsigned int GetImageWidth(unsigned int _index) const;

      /// \brief Gets the height of the image in pixels.
      /// \param[in] _index Index of the camera
      /// \return The image height in pixels.
      /// \sa MultiCameraSensor::GetCameraCount
      public: unsigned int GetImageHeight(unsigned int _index) const;

      /// \brief Gets the raw image data from the sensor.
      /// \param[in] _index Index of the camera
      /// \return The pointer to the image data array.
      /// \sa MultiCameraSensor::GetCameraCount
      public: const unsigned char *GetImageData(unsigned int _index);

      /// \brief Saves the camera image(s) to the disk.
      /// \param[in] _filenames The name of the files for each camera.
      /// \return True if successful, false if unsuccessful.
      /// \sa MultiCameraSensor::GetCameraCount
      public: bool SaveFrame(const std::vector<std::string> &_filenames);

      // Documentation inherited.
      public: virtual bool IsActive();

      // Documentation inherited.
      protected: virtual bool UpdateImpl(bool _force);

      // Documentation inherited.
      protected: virtual void Fini();

      /// \brief Handle the render event.
      private: void Render();

      /// \brief All the cameras.
      private: std::vector<rendering::CameraPtr> cameras;

      /// \brief Mutex to protect the cameras list.
      private: mutable boost::mutex cameraMutex;

      /// \brief Publishes messages of type msgs::ImagesStamped.
      private: transport::PublisherPtr imagePub;

      /// \brief The images msg.
      private: msgs::ImagesStamped msg;

      /// \brief True if the sensor was rendered.
      private: bool rendered;
    };
    /// \}
  }
}
#endif
