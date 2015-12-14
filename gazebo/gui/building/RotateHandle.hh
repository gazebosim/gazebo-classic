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

#ifndef _GAZEBO_BUILDING_ROTATE_HANDLE_HH_
#define _GAZEBO_BUILDING_ROTATE_HANDLE_HH_

#include <memory>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class RotateHandlePrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class RotateHandle RotateHandle.hh
    /// \brief Handle for rotating an editor item
    class GZ_GUI_VISIBLE RotateHandle : public QGraphicsItem
    {
      /// \brief Constructor
      /// param[in] _parent Parent graphics item.
      public: RotateHandle(QGraphicsItem *_parent = 0);

      /// \brief Destructor
      public: ~RotateHandle();

      /// \brief Set the current mouse state.
      /// \param[in] _state Integer corresponding to Qt QEvent type.
      /// http://doc.qt.io/qt-4.8/qevent.html#Type-enum
      /// \sa int MouseState() const
      public: void SetMouseState(int _state);

      /// \brief Get the current mouse state
      /// \return The current mouse state.
      /// \deprecated See int MouseState() const
      public: int GetMouseState() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the X position of the mouse press.
      /// \return Mouse press X position in pixel coordinates.
      /// \deprecated See double MouseDownX() const
      public: double GetMouseDownX() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the Y position of the mouse press.
      /// \return Mouse press Y position in pixel coordinates.
      /// \deprecated See double MouseDownY() const
      public: double GetMouseDownY() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the current mouse state
      /// \return The current mouse state.
      /// \sa void SetMouseState(int _state)
      public: int MouseState() const;

      /// \brief Get the X position of the mouse press.
      /// \return Mouse press X position in pixel coordinates.
      /// \sa void SetMouseDownX(double _x)
      public: double MouseDownX() const;

      /// \brief Get the Y position of the mouse press.
      /// \return Mouse press Y position in pixel coordinates.
      /// \sa void SetMouseDownY(double _y)
      public: double MouseDownY() const;

      /// \brief Set the X position of the mouse press.
      /// \param[in] _x Mouse press X position in pixel coordinates.
      /// \sa double MouseDownX() const
      public: void SetMouseDownX(double _x);

      /// \brief Set the Y position of the mouse press.
      /// \param[in] _y Mouse press Y position in pixel coordinates.
      /// \sa double MouseDownY() const
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

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<RotateHandlePrivate> dataPtr;
    };
    /// \}
  }
}

#endif
