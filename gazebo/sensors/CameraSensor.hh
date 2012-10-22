/*
 * Copyright 2011 Nate Koenig
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
/* Desc: A persepective X11 OpenGL Camera Sensor
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef CAMERASENSOR_HH
#define CAMERASENSOR_HH

#include <string>

#include "sensors/Sensor.hh"
#include "msgs/MessageTypes.hh"
#include "transport/TransportTypes.hh"
#include "rendering/RenderTypes.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors Sensors
    /// \brief A set of sensor classes, functions, and definitions
    /// \{

    /// \brief Basic camera sensor
    /// This sensor is used for simulating standard monocular cameras
    class CameraSensor : public Sensor
    {
      /// \brief Constructor
      public: CameraSensor();

      /// \brief Destructor
      public: virtual ~CameraSensor();

      /// \brief Set the parent of the sensor
      public: virtual void SetParent(const std::string &_name);

      /// \brief Load the camera using parameter from an SDF element
      /// \param _sdf The SDF parameters
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the camera using default parameters
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the camera
      public: virtual void Init();

      /// \brief Returns the name of the camera image topic.
      /// @todo to be implemented
      public: virtual std::string GetTopic() const;

      /// \brief Update the sensor information
      protected: virtual void UpdateImpl(bool _force);

      /// Finalize the camera
      protected: virtual void Fini();

      /// \brief Set whether the sensor is active or not
      public: virtual void SetActive(bool value);

      /// \brief Returns a pointer to the rendering::Camera
      public: rendering::CameraPtr GetCamera() const
              {return this->camera;}

      /// \brief returns the pixel width of an image as an unsigned int
      public: unsigned int GetImageWidth() const;

      /// \brief returns the pixel height of an image as an unsigned int
      public: unsigned int GetImageHeight() const;

      /// \brief returns pixels of an image as an array of unsigned char
      public: const unsigned char *GetImageData();

      /// \brief save the image frame to file named _filename.
      public: bool SaveFrame(const std::string &_filename);

      private: rendering::CameraPtr camera;
      private: rendering::ScenePtr scene;

      private: transport::PublisherPtr imagePub;
    };
    /// \}
  }
}
#endif
