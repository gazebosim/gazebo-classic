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

#ifndef _GAZEBO_BUILDING_WALL_SEGMENT_ITEM_PRIVATE_HH_
#define _GAZEBO_BUILDING_WALL_SEGMENT_ITEM_PRIVATE_HH_

#include "gazebo/gui/building/SegmentItemPrivate.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the WallSegmentItem class
    class WallSegmentItemPrivate : public SegmentItemPrivate
    {
      /// \brief Thickness of the wall segment in the 2d view, in pixels.
      public: double wallThickness;

      /// \brief Height of the wall segment in meters.
      public: double wallHeight;

      /// \brief This wall segment's measure item.
      public: MeasureItem *measure;

      /// \brief Qt action for opening the inspector.
      public: QAction *openInspectorAct;

      /// \brief Qt action for deleting the wall segment item.
      public: QAction *deleteItemAct;

      /// \brief Inspector for configuring the wall segment item.
      public: WallInspectorDialog *inspector;
    };
  }
}
#endif
