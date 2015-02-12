/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _FLOOR_ITEM_HH_
#define _FLOOR_ITEM_HH_

#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingItem;
    class WallSegmentItem;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class FloorItem FloorItem.hh
    /// \brief 2D representation of a floor.
    class GAZEBO_VISIBLE FloorItem : public RectItem, public BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: FloorItem();

      /// \brief Destructor
      public: ~FloorItem();

      // Documentation inherited.
      public: virtual QVector3D GetSize() const;

      // Documentation inherited.
      public: virtual QVector3D GetScenePosition() const;

      // Documentation inherited.
      public: virtual double GetSceneRotation() const;

      /// \brief Attach walls so the floor can auto expand to hold the wall.
      /// \param[in] _wallSegmentItem Wall item to attach to the floor.
      public: void AttachWallSegment(WallSegmentItem *_wallSegmentItem);

      // Documentation inherited.
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      // Documentation inherited.
      private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      // Documentation inherited.
      private: virtual void contextMenuEvent(
          QGraphicsSceneContextMenuEvent *_event);

      /// \brief Qt callback tonNotify that the bounding box of the wall items
      /// needs to be changed.
      private slots: void NotifyChange();

      /// \brief Qt callback to recalculate the bounding box of wall items.
      private slots: void RecalculateBoundingBox();

      /// \brief Qt callback when a wall is being deleted.
      private slots: void WallSegmentDeleted();

      /// \brief Update the floor properties and emit Qt signals
      private: void Update();

      /// \brief Emit floor changed Qt signals.
      public: void FloorChanged();

      /// \brief Emit size changed Qt signals.
      private: void SizeChanged();

      /// \brief Depth of floor item in pixels.
      private: double floorDepth;

      /// \brief Height of floor item in pixels.
      private: double floorHeight;

      /// \brief Width of floor item in pixels.
      private: double floorWidth;

      /// \brief Scene position of floor item in pixel coordinates.
      private: QPointF floorPos;

      /// \brief A flag to indicate whether or not there have been changes to
      /// the wall items.
      private: bool dirty;

      /// \brief A list of wall items that the floor item holds.
      private: std::vector<WallSegmentItem *> wallSegments;

      /// \brief Bounding box of the floor item.
      private: QPolygonF floorBoundingRect;
    };
    /// \}
  }
}

#endif
