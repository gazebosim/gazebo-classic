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

#ifndef _CORNER_GRABBER_HH_
#define _CORNER_GRABBER_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class CornerGrabber : public QGraphicsItem
    {
      public: explicit CornerGrabber(QGraphicsItem *_parent = 0, int index = 0);

      public: int GetIndex();

      /// \brief Set the current mouse state
      public: void SetMouseState(int _state);

      /// \brief Retrieve the current mouse state
      public: int  GetMouseState();

      public: QPointF GetCenterPoint();

      public: void SetMouseDownX(double _x);

      public: void SetMouseDownY(double _y);

      public: double GetMouseDownX();

      public: double GetMouseDownY();

      public: void SetWidth(double _width);

      public: void SetHeight(double _height);

      public: double GetWidth();

      public: double GetHeight();

      public: void SetColor(QColor color);

      public: QColor GetColor();

      /// \brief Mouse states
      public: enum mouseStates {kMouseReleased=0, kMouseDown, kMouseMoving};

      public: virtual QRectF boundingRect() const;

      private: virtual void paint (QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      protected: void hoverEnterEvent (QGraphicsSceneHoverEvent *_event);

      protected: void hoverLeaveEvent (QGraphicsSceneHoverEvent *_event);

      protected: void mouseMoveEvent (QGraphicsSceneMouseEvent *_event);

      protected: void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

      protected: void mousePressEvent (QGraphicsSceneMouseEvent *_event);

      protected: void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

      protected: void mouseReleaseEvent (QGraphicsSceneMouseEvent *_event);

      private: int index;

      private: double mouseDownX;

      private: double mouseDownY;

      /// \brief the hover event handlers will toggle this between red and black
      private: QColor borderColor;

      private: double width;

      private: double height;

      private: double widthGrabBuffer;

      private: double heightGrabBuffer;

      private: int mouseButtonState;

      private: double size;
    };
  }
}

#endif // CORNERGRABBER_H
