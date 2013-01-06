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

#ifndef _ROTATE_HANDLE_HH_
#define _ROTATE_HANDLE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class RotateHandle : public QGraphicsItem
    {
      public: RotateHandle(QGraphicsItem *_parent = 0);

      public: ~RotateHandle();

      /// \brief Set the current mouse state
      public: void SetMouseState(int _state);

      /// \brief Retrieve the current mouse state
      public: int  GetMouseState() const;

      public: void SetMouseDownX(double _x);

      public: void SetMouseDownY(double _y);

      public: double GetMouseDownX() const;

      public: double GetMouseDownY() const;

      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      public: virtual QRectF boundingRect() const;

      private: void paint (QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      private: int mouseButtonState;

      private: double mouseDownX;

      private: double mouseDownY;

      private: QColor borderColor;

      private: double handleSize;

      private: double handleOffsetHeight;

      private: QPointF origin;

      private: QPointF handleOffset;


    };
  }
}

#endif
