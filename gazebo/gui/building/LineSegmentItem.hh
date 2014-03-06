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

#ifndef _LINE_SEGMENT_ITEM_HH_
#define _LINE_SEGMENT_ITEM_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class EditorItem;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class LineSegmentItem LineSegmentItem.hh
    /// \brief 2D line segment.
    class GAZEBO_VISIBLE LineSegmentItem
      : public EditorItem, public QGraphicsLineItem
    {
      /// \brief Constructor
      /// \param[in] _parent Parent graphics item.
      /// \param[in] _index Index of the line segment in polyline.
      public: LineSegmentItem(QGraphicsItem *_parent = 0, int _index = 0);

      /// \brief Destructor
      public: ~LineSegmentItem();

      /// \brief Set the line
      /// \param[in] _start Start position of the line in pixel coordinates.
      /// \param[in] _end End position of the line in pixel coordinates.
      public: void SetLine(const QPointF &_start, const QPointF &_end);

      /// \brief Set the start position of the line
      /// \param[in] _start Start position of the line in pixel coordinates.
      public: void SetStartPoint(const QPointF &_start);

      /// \brief Set the end position of the line
      /// \param[in] _end End position of the line in pixel coordinates.
      public: void SetEndPoint(const QPointF &_end);

      /// \brief Get the index of the line.
      /// \return The index of the line.
      public: int GetIndex() const;

      // Documentation Inherited
      public: QVector3D GetSize() const;

      // Documentation Inherited
      public: QVector3D GetScenePosition() const;

      // Documentation Inherited
      public: double GetSceneRotation() const;

      /// \brief Set the current mouse state.
      /// \param[in] _state Mouse state.
      public: void SetMouseState(int _state);

      /// \brief Get the current mouse state
      public: int  GetMouseState() const;

      /// \brief Get the X position of the mouse press.
      /// \return Mouse press X position in pixel coordinates.
      public: double GetMouseDownX() const;

      /// \brief Get the Y position of the mouse press.
      /// \return Mouse press Y position in pixel coordinates.
      public: double GetMouseDownY() const;

      /// \brief Set the X position of the mouse press.
      /// \param[in] _x Mouse press X position in pixel coordinates.
      public: void SetMouseDownX(double _x);

      /// \brief Set the Y position of the mouse press.
      /// \param[in] _y Mouse press Y position in pixel coordinates.
      public: void SetMouseDownY(double _y);

      /// \brief Update by emtting the LineChanged signal.
      public: void Update();

      /// \brief React to item changes notified by Qt.
      /// \param[in] _change Qt change type, e.g. selected change
      /// \param[in] _value Value to be changed to.
      private: QVariant itemChange(GraphicsItemChange _change,
          const QVariant &_value);

      /// \brief Qt mouse hover enter event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover move event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover leave event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      private: void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

      /// \brief Qt mouse move event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      private: void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

      /// \brief Qt paint function for drawing the line segment.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
      private: void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Emit line changed Qt signals.
      private: void LineChanged();

      /// \brief Index of line segment in polyline.
      private: int index;

      /// \brief Line segment's start position in pixel coordinates.
      private: QPointF start;

      /// \brief Line segment's end position in pixel coordinates.
      private: QPointF end;

      /// \brief The current mouse state.
      private: int mouseButtonState;

      /// \brief Mouse press X position in pixel coordinates.
      private: double mouseDownX;

      /// \brief Mouse press Y position in pixel coordinates.
      private: double mouseDownY;
    };
    /// \}
  }
}

#endif
