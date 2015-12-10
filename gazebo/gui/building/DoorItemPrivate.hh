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

#ifndef _GAZEBO_BUILDING_DOOR_ITEM_PRIVATE_HH_
#define _GAZEBO_BUILDING_DOOR_ITEM_PRIVATE_HH_

#include "gazebo/gui/building/RectItemPrivate.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the DoorItem class
    class DoorItemPrivate : public RectItemPrivate
    {
      /// \brief Door depth in pixels
      public: double doorDepth;

      /// \brief Door height in pixels
      public: double doorHeight;

      /// \brief Door width in pixels
      public: double doorWidth;

      /// \brief Door elevation in pixels
      public: double doorElevation;

      /// \brief Door scene position in pixel coordinates.
      public: QPointF doorPos;

      /// \brief Inspector for configuring the door item.
      public: WindowDoorInspectorDialog *inspector;
    };
  }
}
#endif
