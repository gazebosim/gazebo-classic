/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _WALL_SEGMENT_ITEM_HH_
#define _WALL_SEGMENT_ITEM_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/MeasureItem.hh"
#include "gazebo/gui/building/SegmentItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"

namespace gazebo
{
  namespace gui
  {
    class MeasureItem;
    class SegmentItem;
    class BuildingItem;
    class WallInspectorDialog;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class WallSegmentItem WallSegmentItem.hh
    /// \brief 2D representation of a wall.
    class GAZEBO_VISIBLE WallSegmentItem : public SegmentItem, public
        BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      /// param[in] _start Start position of the item in pixel coordinates.
      /// param[in] _end End position of the item in pixel coordinates.
      /// param[in] _height Height of the wall segment in meters.
      public: WallSegmentItem(const QPointF &_start, const QPointF &_end,
          const double _height);

      /// \brief Destructor
      public: ~WallSegmentItem();

      /// \brief Get the height of the wall segment item.
      /// \return Height of the wall segment item in pixels.
      public: double GetHeight() const;

      /// \brief Set the height of the wall segment item.
      /// param[in] _height Height of the wall segment item in pixels.
      public: void SetHeight(double _height);

      /// \brief Clone the wall segment item.
      /// \return A pointer to a copy of the wall segment item.
      public: WallSegmentItem *Clone() const;

      /// \brief Update by emitting Qt signals.
      public: void Update();

      /// \brief Emit wall segment changed Qt signals.
      public: void WallSegmentChanged();

      /// \brief Update inspector with current values.
      public: void UpdateInspector();

      // Documentation inherited
      public: void SetHighlighted(bool _highlighted);

      /// \brief Update wall segment when segment updated.
      protected: void SegmentUpdated();

      /// \brief Qt context menu event received on a mouse right click.
      /// \param[in] _event Qt context menu event.
      private: void contextMenuEvent(QGraphicsSceneContextMenuEvent *_event);

      /// \brief Qt context menu event received on a mouse double click.
      /// \param[in] _event Qt double click event.
      private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *
          _event);

      /// \brief React to item changes notified by Qt.
      /// \param[in] _change Qt change type, e.g. selected change
      /// \param[in] _value Value to be changed to.
      private: QVariant itemChange(GraphicsItemChange _change,
          const QVariant &_value);

      // Documentation inherited
      private slots: void OnApply();

      // Documentation inherited
      private slots: void OnOpenInspector();

      // Documentation inherited
      private slots: void OnDeleteItem();

      /// \brief Thickness of the wall segment in the 2d view, in pixels.
      private: double wallThickness;

      /// \brief Height of the wall segment in meters.
      private: double wallHeight;

      /// \brief Scale for converting pixels to metric units.
      private: double scale;

      /// \brief This wall segment's measure item.
      private: MeasureItem *measure;

      /// \brief Qt action for opening the inspector.
      private: QAction *openInspectorAct;

      /// \brief Qt action for deleting the wall segment item.
      private: QAction *deleteItemAct;

      /// \brief Inspector for configuring the wall segment item.
      private: WallInspectorDialog *inspector;
    };
    /// \}
  }
}

#endif
