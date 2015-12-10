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

#ifndef _GAZEBO_BUILDING_SEGMENT_ITEM_PRIVATE_HH_
#define _GAZEBO_BUILDING_SEGMENT_ITEM_PRIVATE_HH_

#include "gazebo/gui/building/EditorItemPrivate.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the SegmentItem class
    class SegmentItemPrivate : public EditorItemPrivate
    {
      /// \brief A list of grabber handles for this item. One grabber for each
      /// endpoint.
      public: std::vector<GrabberHandle *> grabbers;

      /// \brief Segment's start position in pixel coordinates.
      public: QPointF start;

      /// \brief Segment's end position in pixel coordinates.
      public: QPointF end;

      /// \brief Keep track of mouse press position for translation.
      public: QPointF segmentMouseMove;

      /// \brief Thickness of the segment on the 2d view, in pixels.
      public: double thickness;

      /// \brief Width of grabbers in pixels.
      public: double grabberWidth;

      /// \brief Height of grabbers in pixels.
      public: double grabberHeight;
    };
  }
}
#endif
