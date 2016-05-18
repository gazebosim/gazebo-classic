/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_BUILDING_WINDOWITEM_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_WINDOWITEM_PRIVATE_HH_

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class WindowDoorInspectorDialog;

    /// \brief Private data for the WindowItem class
    class WindowItemPrivate
    {
      /// \brief Depth of the window item in pixels.
      public: double windowDepth;

      /// \brief Height of the window item in pixels.
      public: double windowHeight;

      /// \brief Width of the window item in pixels.
      public: double windowWidth;

      /// \brief Side bar of the window item in pixels.
      public: double windowSideBar;

      /// \brief Scene position of the window item in pixel coordinates.
      public: ignition::math::Vector2d windowPos;

      /// \brief Elevation of the window item in pixels.
      public: double windowElevation;

      /// \brief Inspector for configuring the window item.
      public: WindowDoorInspectorDialog *inspector;
    };
  }
}
#endif
