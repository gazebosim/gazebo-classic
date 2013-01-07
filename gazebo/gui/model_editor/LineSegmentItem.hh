/*
 * Copyright 2012 Open Source Robotics Foundation
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

namespace gazebo
{
  namespace gui
  {

    class EditorItem;

    class LineSegmentItem : public EditorItem, public QGraphicsLineItem
    {
      public: LineSegmentItem(QGraphicsItem *_parent = 0, int _index = 0);

      public: ~LineSegmentItem();

      public: void SetLine(const QPointF &_start, const QPointF &_end);

      public: void SetStartPoint(const QPointF &_start);

      public: void SetEndPoint(const QPointF &_end);

      public: int GetIndex() const;

      public: QVector3D GetSize() const;

      public: QVector3D GetScenePosition() const;

      public: double GetSceneRotation() const;

      /// \brief Set the current mouse state
      public: void SetMouseState(int _state);

      /// \brief Retrieve the current mouse state
      public: int  GetMouseState() const;

      public: void SetMouseDownX(double _x);

      public: void SetMouseDownY(double _y);

      public: double GetMouseDownX() const;

      public: double GetMouseDownY() const;

      public: void Update();

      private: QVariant itemChange(GraphicsItemChange _change,
          const QVariant &_value);

      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      private: void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      private: void LineChanged();

      private: int index;

      private: QPointF start;

      private: QPointF end;

      private: int mouseButtonState;

      private: double mouseDownX;

      private: double mouseDownY;
    };
  }
}

#endif
