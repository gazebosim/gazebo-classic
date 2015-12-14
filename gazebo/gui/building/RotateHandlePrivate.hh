/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_BUILDING_ROTATE_HANDLE_PRIVATE_HH_
#define _GAZEBO_BUILDING_ROTATE_HANDLE_PRIVATE_HH_

#include <ignition/math/Vector2.hh>

#include "gazebo/common/Color.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the RotateHandle class
    class RotateHandlePrivate
    {
      /// \brief Current mouse state.
      public: int mouseButtonState;

      /// \brief Mouse press X position in pixel coordinates.
      public: double mouseDownX;

      /// \brief Mouse press Y position in pixel coordinates.
      public: double mouseDownY;

      /// \brief Border color of the rotate handle.
      public: common::Color borderColor;

      /// \brief Size of the rotate handle in pixels.
      public: double handleSize;

      /// \brief Offset height of the rotate handle (from the item) in pixels.
      public: double handleOffsetHeight;

      /// \brief Origin of the rotate handle.
      public: ignition::math::Vector2d origin;

      /// \brief Offset position of the rotate handle in pixel coordinates.
      public: ignition::math::Vector2d handleOffset;
    };
  }
}

#endif
