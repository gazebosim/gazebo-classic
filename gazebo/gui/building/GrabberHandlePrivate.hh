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

#ifndef _GAZEBO_GUI_GRABBER_HANDLE_PRIVATE_HH_
#define _GAZEBO_GUI_GRABBER_HANDLE_PRIVATE_HH_

#include <vector>

#include "gazebo/common/Color.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;

    /// \internal
    /// \brief Private data for GrabberHandle
    class GrabberHandlePrivate
    {
      /// \brief A list of grabbers linked to this grabber.
      public: std::vector<GrabberHandle *> linkedGrabbers;

      /// \brief Index of this corner grabber.
      public: int index;

      /// \brief Mouse press X position in pixel coordinates.
      public: double mouseDownX;

      /// \brief Mouse press Y position in pixel coordinates.
      public: double mouseDownY;

      /// \brief Fill color of the grabber handle.
      public: common::Color handleColor;

      /// \brief Border color of the grabber handle.
      public: common::Color borderColor;

      /// \brief Width of the grabber handle in pixels.
      public: double width;

      /// \brief Height of the grabber handle in pixels.
      public: double height;

      /// \brief Extra width around the grabber handle for mouse grabbing.
      public: double widthGrabBuffer;

      /// \brief Extra height around the grabber handle for mouse grabbing.
      public: double heightGrabBuffer;

      /// \brief Current mouse state.
      public: int mouseButtonState;
    };
  }
}

#endif
