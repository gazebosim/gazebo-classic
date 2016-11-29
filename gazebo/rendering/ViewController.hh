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
#ifndef GAZEBO_RENDERING_VIEWCONTROLLER_HH_
#define GAZEBO_RENDERING_VIEWCONTROLLER_HH_

#include <string>
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class MouseEvent;
  }

  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class ViewController ViewController.hh rendering/rendering.hh
    /// \brief Base class for view controllers.
    class GZ_RENDERING_VISIBLE ViewController
    {
      /// \brief Constructor
      /// \param[in] _camera The user camera to controll.
      public: explicit ViewController(UserCameraPtr _camera);

      /// \brief Destructor
      public: virtual ~ViewController();

      /// \brief Initialize the view controller.
      public: virtual void Init() = 0;

      /// \brief Initialize with a focus point.
      /// \param[in] _focalPoint The point to look at.
      /// \param[in] _yaw Initial yaw angle.
      /// \param[in] _pitch Initial pitch angle.
      public: virtual void Init(const math::Vector3 &_focalPoint,
                  const double _yaw = 0, const double _pitch = 0);

      /// \brief Update the controller, which should update the position
      /// of the Camera.
      public: virtual void Update() = 0;

      /// \brief Called by the UserCamera when a resize event occurs.
      /// \param[in] _width New width
      /// \param[in] _height New height
      public: virtual void Resize(const unsigned int _width,
                                  const unsigned int _height);

      /// \brief Set whether the controller is enabled.
      /// \param[in] _value True if the controller is enabled.
      public: void SetEnabled(bool _value);

      /// \brief Handle a mouse event.
      /// \param[in] _event The mouse position.
      public: virtual void HandleMouseEvent(
                  const common::MouseEvent &_event) = 0;

      /// \brief Handle a key release event.
      /// \param[in] _key The key that was released.
      public: virtual void HandleKeyReleaseEvent(const std::string &_key) = 0;

      /// \brief Handle a key press event
      /// \param[in] _key The key that was pressed.
      public: virtual void HandleKeyPressEvent(const std::string &_key) = 0;

      /// \brief Get the type of view controller.
      /// \return The view controller type string.
      public: std::string GetTypeString() const;

      /// \brief Pointer to the camera to control.
      protected: UserCameraPtr camera;

      /// \brief True if enabled.
      protected: bool enabled;

      /// \brief Type of view controller.
      protected: std::string typeString;
    };
    /// \}
  }
}
#endif
