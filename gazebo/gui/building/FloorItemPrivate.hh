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

#ifndef _GAZEBO_BUILDING_FLOOR_ITEM_PRIVATE_HH_
#define _GAZEBO_BUILDING_FLOOR_ITEM_PRIVATE_HH_

#include "gazebo/gui/building/RectItemPrivate.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the FloorItem class
    class FloorItemPrivate : public RectItemPrivate
    {
      /// \brief Depth of floor item in pixels.
      public: double floorDepth;

      /// \brief Height of floor item in pixels.
      public: double floorHeight;

      /// \brief Width of floor item in pixels.
      public: double floorWidth;

      /// \brief Scene position of floor item in pixel coordinates.
      public: QPointF floorPos;

      /// \brief A flag to indicate whether or not there have been changes to
      /// the wall items.
      public: bool dirty;

      /// \brief A list of wall items that the floor item holds.
      public: std::vector<WallSegmentItem *> wallSegments;

      /// \brief Bounding box of the floor item.
      public: QPolygonF floorBoundingRect;
    };
  }
}
#endif
