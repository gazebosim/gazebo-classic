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

#ifndef _GAZEBO_BUILDING_RECT_ITEM_PRIVATE_HH_
#define _GAZEBO_BUILDING_RECT_ITEM_PRIVATE_HH_

#include "gazebo/gui/building/MeasureItem.hh"
#include "gazebo/gui/building/RotateHandle.hh"
#include "gazebo/gui/building/EditorItemPrivate.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the RectItem class
    class RectItemPrivate : public EditorItemPrivate
    {
      /// \brief Width of rect item in pixels.
      public: double width;

      /// \brief Height of rect item in pixels.
      public: double height;

      /// \brief Actual width of rect item drawn in pixels.
      public: double drawingWidth;

      /// \brief Actual height of rect item drawn in pixels.
      public: double drawingHeight;

      /// \brief X origin of the rect item in pixels.
      public: double drawingOriginX;

      /// \brief Y origin of the rect item in pixels.
      public: double drawingOriginY;

      /// \brief Border color of the rect item.
      public: QColor borderColor;

      /// \brief Rotation angle of the rect item in degrees.
      public: double rotationAngle;

      /// \brief Qt action for opening the inspector.
      public: QAction *openInspectorAct;

      /// \brief Qt action for deleting the item.
      public: QAction *deleteItemAct;

      /// \brief A vector containing this item's measure items.
      /// Currently only used for windows and doors, containing one measure
      /// towards each end of this item's parent wall.
      public: std::vector<MeasureItem *> measures;

      /// \brief Mouse press position in pixel coordinates.
      public: QPointF mousePressPos;

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
