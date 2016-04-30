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

#ifndef _GAZEBO_GUI_BUILDING_RECTITEM_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_RECTITEM_PRIVATE_HH_

#include <vector>
#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;
    class RotateHandle;

    /// \brief Private data for the RectItem class
    class RectItemPrivate
    {
      /// \brief Mouse press position in pixel coordinates.
      public: ignition::math::Vector2d mousePressPos;

      /// \brief Mouse press position in pixel coordinates.
      public: int gridSpace;

      /// \brief A list of grabber handles for this item. Four for corners and
      /// four for edges, going clockwise with 0 being top left
      public: std::vector<GrabberHandle *> grabbers;

      /// \brief Rotate handle for rotating the rect item.
      public: RotateHandle *rotateHandle;

      /// \brief A list of resize cursors used when the mouse hovers over the
      /// grabber handles.
      public: std::vector<Qt::CursorShape> cursors;

      /// \brief Resize flag that controls how the rect item can be resized.
      public: unsigned int resizeFlag;

      /// \brief Normalized position with respect to the wall segment's start
      /// point.
      public: double positionOnWall;

      /// \brief Angle with respect to parent wall, either 0 or 180 degrees.
      public: double angleOnWall;
    };
  }
}
#endif
