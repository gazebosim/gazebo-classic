/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Camera Visualization Class
 * Author: Nate Koenig
 */

#ifndef CAMERAVISUAL_HH
#define CAMERAVISUAL_HH

#include <string>

#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    class Camera;

    /// \class CameraVisual CameraVisual.hh rendering/rendering.hh
    /// \brief Basic camera visualization
    ///
    /// This class is used to visualize a camera image generated from
    /// a CameraSensor. The sensor's image is drawn on a billboard in the 3D
    /// environment.
    class CameraVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the Visual
      /// \param[in] _vis Pointer to the parent Visual
      public: CameraVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~CameraVisual();

      /// \brief Load the Visual
      /// \param[in] _width Width of the Camera image
      /// \param[in] _height Height of the Camera image
      public: void Load(unsigned int _width, unsigned int _height);

      private: CameraPtr camera;
    };
    /// \}
  }
}
#endif
