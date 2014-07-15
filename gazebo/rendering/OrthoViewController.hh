/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _ORTHOVIEWCONTROLLER_HH_
#define _ORTHOVIEWCONTROLLER_HH_

#include <string>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ViewController.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class OrthoViewController OrbitVeiwController.hh rendering/rendering.hh
    /// \brief Orthographic view controller
    class GAZEBO_VISIBLE OrthoViewController : public OrbitViewController
    {
      /// \brief Constructor.
      /// \param[in] _camera Pointer to the camera to control.
      public: OrthoViewController(UserCameraPtr _camera);

      /// \brief Destructor.
      public: virtual ~OrthoViewController();

      /// \brief Initialize the controller.
      public: virtual void Init();

      /// \brief Handle a mouse event.
      /// \param[in] _event The mouse event.
      public: virtual void HandleMouseEvent(const common::MouseEvent &_event);

      /// \brief Zoom the camera.
      /// \param[in] _amount Zoom quatity.
      /// \param[in] _screenPos Position on screen to zoom to
      private: void Zoom(float _amount,
          math::Vector2i _point = math::Vector2i(0, 0));

      /// \brief Scale of the orthographic view window
      private: double scale;
    };
    /// \}
  }
}
#endif
