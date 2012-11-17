/*
 * Copyright 2012 Nate Koenig
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

#ifndef _DEPTHCAMERASENSOR_HH_
#define _DEPTHCAMERASENSOR_HH_

#include <string>

#include "sensors/Sensor.hh"
#include "msgs/MessageTypes.hh"
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

    /// \class DepthCameraSensor DepthCameraSensor.hh sensors/sensors.hh
    /// \brief Depth camera sensor
    /// This sensor is used for simulating standard monocular cameras
    class DepthCameraSensor : public Sensor
    {
      /// \brief Constructor
      public: DepthCameraSensor();

      /// \brief Destructor
      public: virtual ~DepthCameraSensor();

      // Documentation Inherited
      public: virtual void SetParent(const std::string &_name);

      // Documentation Inherited
      protected: virtual void Load(const std::string &_worldName,
                                   sdf::ElementPtr &_sdf);

      // Documentation Inherited
      protected: virtual void Load(const std::string &_worldName);

      // Documentation Inherited
      protected: virtual void Init();

      // Documentation Inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation Inherited
      protected: virtual void Fini();

      // Documentation Inherited
      public: virtual void SetActive(bool _value);

      /// \brief Returns a pointer to the rendering::DepthCamera
      /// \return Depth Camera pointer
      public: rendering::DepthCameraPtr GetDepthCamera() const
              {return this->camera;}

      /// \brief Saves an image frame of depth camera sensor to file
      /// \param[in] _filename Name of file to save as
      /// \return True if saved, false if not
      public: bool SaveFrame(const std::string &_filename);

      private: rendering::DepthCameraPtr camera;

      private: rendering::ScenePtr scene;
    };
    /// \}
  }
}
#endif


