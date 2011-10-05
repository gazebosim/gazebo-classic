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

#ifndef DEPTHCAMERASENSOR_HH
#define DEPTHCAMERASENSOR_HH

#include "sensors/Sensor.hh"
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
    class DepthCameraSensor : public Sensor
    {
      /// \brief Constructor
      public: DepthCameraSensor();
    
      /// \brief Destructor
      public: virtual ~DepthCameraSensor();
    
      /// \brief Load the camera using parameter from an SDF element
      /// \param _sdf The SDF parameters
      protected: virtual void Load( sdf::ElementPtr &_sdf );

      /// \brief Load the camera using default parameters
      protected: virtual void Load();
    
      /// \brief Initialize the camera
      protected: virtual void InitChild();
    
      /// \brief Update the sensor information
      protected: virtual void Update(bool force);
    
      /// Finalize the camera
      protected: virtual void Fini();
    
      /// \brief Set whether the sensor is active or not
      public: virtual void SetActive(bool value);
    
      private: void OnPose(const boost::shared_ptr<msgs::Pose const> &_msg);
      private: void Render();
    
      private: rendering::CameraPtr camera;
      public: rendering::CameraPtr GetCamera() const {return this->camera; };
    
      protected: std::string ogreTextureName;
    };
    /// \}
  }
}
#endif
