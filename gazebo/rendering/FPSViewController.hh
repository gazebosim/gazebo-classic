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
#ifndef _FPSVIEWCONTROLLER_HH_
#define _FPSVIEWCONTROLLER_HH_

#include <string>

#include "rendering/ViewController.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class FPSViewController FPSViewController.hh rendering/rendering.hh
    /// \brief First Person Shooter style view controller
    class FPSViewController : public ViewController
    {
      /// \brief Constructor
      /// \param[in] Camera to controll
      public: FPSViewController(UserCameraPtr _camera);

      /// \brief Destructor
      public: virtual ~FPSViewController();

      /// \brief Initialize the controller
      public: virtual void Init();

      /// \brief Update the camera position
      public: virtual void Update();

      /// \brief Get the type name of this view controller
      /// \return The name of the controller type: "fps"
      public: static std::string GetTypeString();

      /// \brief Handle a mouse event
      ///
      /// This is ususally called from the graphical interface.
      /// \param[in] _event The mouse event.
      public: virtual void HandleMouseEvent(const common::MouseEvent &_event);
    };
    /// \}
  }
}
#endif
