/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "sensors/Sensor.hh"

namespace gazebo
{
  namespace rendering
  {
    class Camera;
  }

  namespace sensors
  {
    /// \brief Basic camera sensor
    ///
    /// This sensor is used for simulating standard monocular cameras
    class CameraSensor : public Sensor
    {
      /// \brief Constructor
      public: CameraSensor();
    
      /// \brief Destructor
      public: virtual ~CameraSensor();
    
      /// \brief Load the camera using parameter from an XMLConfig node
      /// \param node The XMLConfig node
      protected: virtual void Load( boost::shared_ptr<sdf::SDFElement> _sdf );
    
      /// \brief Initialize the camera
      protected: virtual void Init();
    
      /// \brief Update the sensor information
      protected: virtual void Update(bool force);
    
      /// Finalize the camera
      protected: virtual void Fini();
    
      /// \brief Set whether the sensor is active or not
      public: virtual void SetActive(bool value);
    
      private: void Render();
    
      private: rendering::CameraPtr camera;
    
      protected: std::string ogreTextureName;
    };

  }
}
#endif
