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

#ifndef _ROTATE_HANDLE_HH_
#define _ROTATE_HANDLE_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class RotateHandle RotateHandle.hh
    /// \brief Handle for rotating an editor item
    class GZ_GUI_BUILDING_VISIBLE RotateHandle : public QGraphicsItem
    {
      /// \brief Constructor
      /// param[in] _parent Parent graphics item.
      public: RotateHandle(QGraphicsItem *_parent = 0);

      /// \brief Destructor
      public: ~RotateHandle();

      /// \brief Set the current mouse state
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

      /// \brief Qt mouse hover enter event
      /// \param[in] _event Qt mouse hover event
      protected: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover leave event
      /// \param[in] _event Qt mouse hover event
      protected: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse move event
      /// \param[in] _event Qt mouse event
      protected: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event
      /// \param[in] _event Qt mouse event
      protected: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse release event
      /// \param[in] _event Qt mouse event
      protected: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Get the bounding box of the rotate handle.
      /// \return The grabber handle bounding box.
      public: virtual QRectF boundingRect() const;

      /// \brief Qt paint function for drawing the grabber handle.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
      private: void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Current mouse state.
      private: int mouseButtonState;

      /// \brief Mouse press X position in pixel coordinates.
      private: double mouseDownX;

      /// \brief Mouse press Y position in pixel coordinates.
      private: double mouseDownY;

      /// \brief Border color of the rotate handle.
      private: QColor borderColor;

      /// \brief Size of the rotate handle in pixels.
      private: double handleSize;

      /// \brief Offset height of the rotate handle (from the item) in pixels.
      private: double handleOffsetHeight;

      /// \brief Origin of the rotate handle.
      private: QPointF origin;

      /// \brief Offset position of the rotate handle in pixel coordinates.
      private: QPointF handleOffset;
    };
    /// \}
  }
}

#endif
