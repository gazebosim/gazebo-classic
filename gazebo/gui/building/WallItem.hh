/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _WALL_ITEM_HH_
#define _WALL_ITEM_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/PolylineItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class PolylineItem;
    class GrabberHandle;
    class LineSegmentItem;
    class BuildingItem;
    class WallInspectorDialog;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class WallItem WallItem.hh
    /// \brief 2D representation of a wall.
    class GAZEBO_VISIBLE WallItem : public PolylineItem, public BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      /// param[in] _start Start position of the wall item in pixel coordinates.
      /// param[in] _end End position of the wall item in pixel coordinates.
      /// param[in] _height Height of the wall in meters.
      public: WallItem(const QPointF &_start, const QPointF &_end,
          const double _height);

      /// \brief Destructor
      public: ~WallItem();

      /// \brief Get the height of the wall item.
      /// \return Height of the wall item in pixels.
      public: double GetHeight() const;

      /// \brief Set the height of the wall item.
      /// param[in] _height Height of the wall item in pixels.
      public: void SetHeight(double _height);

      /// \brief Clone the wall item.
      /// \return A pointer to a copy of the wall item.
      public: WallItem *Clone() const;

      /// \brief Update by emitting wall changed Qt signals.
      public: void Update();

      // Documentation inherited
      private: bool GrabberEventFilter(GrabberHandle *_grabber,
          QEvent *_event);

      // Documentation inherited
      private: bool SegmentEventFilter(LineSegmentItem *_segment,
          QEvent *_event);

      /// \brief Qt context menu event received on a mouse right click.
      /// \param[in] Qt context menu event.
      private: void contextMenuEvent(QGraphicsSceneContextMenuEvent *_event);

      /// \brief Helper function for updating the pose of children items
      /// attached to a wall segment.
      /// \param[in] _segment Parent segment
      private: void UpdateSegmentChildren(LineSegmentItem *_segment);

      // Documentation inherited
      private slots: void OnApply();

      // Documentation inherited
      private slots: void OnOpenInspector();

      // Documentation inherited
      private slots: void OnDeleteItem();

      /// \brief Emit wall changed Qt signals.
      public: void WallChanged();

      /// \brief Set a particular segment of the wall to be selected or not.
      /// \param[in] _index Index of the wall segment.
      /// \param[in] _selected True to be in a selected state, false to
      /// disable interaction with the wall segment.
      private: void SetSegmentSelected(unsigned int _index, bool _selected);

      /// \brief Thickness of the wall in pixels.
      private: double wallThickness;

      /// \brief Height of the wall in pixels.
      private: double wallHeight;

      /// \brief Scale for converting pixels to metric units.
      private: double scale;

      /// \brief The current selected segment of the wall.
      private: LineSegmentItem* selectedSegment;

      /// \brief Qt action for opening the inspector.
      private: QAction *openInspectorAct;

      /// \brief Qt action for deleting the wall item.
      private: QAction *deleteItemAct;

      /// \brief Inspector for configuring the wall item.
      private: WallInspectorDialog *inspector;
    };
    /// \}
  }
}

#endif
