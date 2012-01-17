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
#ifndef FPSVIEWCONTROLLER_HH
#define FPSVIEWCONTROLLER_HH

#include "rendering/ViewController.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{
    /// \brief First Person Shooter style view controller
    class FPSViewController : public ViewController
    {
      /// \brief Constructor
      public: FPSViewController(UserCamera *camera);

      /// \brief Destructor
      public: virtual ~FPSViewController();

      public: virtual void Init();

      /// \brief Update
      public: virtual void Update();

      /// \brief Get the type name of this view controller
      public: static std::string GetTypeString();

      /// \brief Handle a mouse event
      public: virtual void HandleMouseEvent(const common::MouseEvent &event);
    };
    /// \}
  }
}
#endif

