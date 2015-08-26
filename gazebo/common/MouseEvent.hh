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
#ifndef _GAZEBO_MOUSEEVENT_HH_
#define _GAZEBO_MOUSEEVENT_HH_

#include <ignition/math/Vector2.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// Forward declare private data
    class MouseEventPrivate;

    /// \addtogroup gazebo_common
    /// \{

    /// \class MouseEvent MouseEvent.hh common/common.hh
    /// \brief Generic description of a mouse event.
    class GZ_COMMON_VISIBLE MouseEvent
    {
      /// \brief Standard mouse buttons enumeration.
      public: enum MouseButton
              {
                /// \brief No button
                NO_BUTTON = 0x0,

                /// \brief Left button
                LEFT = 0x1,

                /// \brief Middle button
                MIDDLE = 0x2,

                /// \brief Right button
                RIGHT = 0x4
              };

      /// \brief Mouse event types enumeration.
      public: enum EventType
              {
                /// \brief No event
                NO_EVENT,

                /// \brief Move event
                MOVE,

                /// \brief Press event
                PRESS,

                /// \brief Release event
                RELEASE,

                /// \brief Scroll event
                SCROLL
              };

      /// \brief Constructor.
      public: MouseEvent();

      /// \brief Copy constructor.
      /// \param[in] _other Other mouse event
      public: MouseEvent(const MouseEvent &_other);

      /// \brief Destructor
      public: virtual ~MouseEvent();

      /// \brief Get mouse pointer position on the screen.
      /// \return Mouse pointer position on the screen.
      public: ignition::math::Vector2i Pos() const;

      /// \brief Set mouse pointer position on the screen.
      /// \param[in] _pos Mouse pointer position on the screen.
      public: void SetPos(const ignition::math::Vector2i &_pos);

      /// \brief Set mouse pointer position on the screen.
      /// \param[in] _x Mouse pointer X position on the screen.
      /// \param[in] _y Mouse pointer Y position on the screen.
      public: void SetPos(const int _x, const int _y);

      /// \brief Get the previous position.
      /// \return The previous mouse position.
      public: ignition::math::Vector2i PrevPos() const;

      /// \brief Set the previous position.
      /// \param[in] _pos Previous mouse pointer position on the screen.
      public: void SetPrevPos(const ignition::math::Vector2i &_pos);

      /// \brief Set the previous position.
      /// \param[in] _x Previous mouse pointer X position on the screen.
      /// \param[in] _y Previous mouse pointer Y position on the screen.
      public: void SetPrevPos(const int _x, const int _y);

      /// \brief Get the position of button press.
      /// \return Screen position of a button press.
      public: ignition::math::Vector2i PressPos() const;

      /// \brief Set the position of button press.
      /// \param[in] _pos Mouse pointer position on the screen.
      public: void SetPressPos(const ignition::math::Vector2i &_pos);

      /// \brief Set the position of button press.
      /// \param[in] _x Mouse pointer X position on the screen.
      /// \param[in] _y Mouse pointer Y position on the screen.
      public: void SetPressPos(const int _x, const int _y);

      /// \brief Get the scroll position.
      /// \return The scroll position.
      public: ignition::math::Vector2i Scroll() const;

      /// \brief Set the scroll position.
      /// \param[in] _scroll Scroll position.
      public: void SetScroll(const ignition::math::Vector2i &_scroll);

      /// \brief Set the scroll position.
      /// \param[in] _x Scroll X position.
      /// \param[in] _y Scroll Y position.
      public: void SetScroll(const int _x, const int _y);

      /// \brief Get the scaling factor.
      /// \return The move scaling factor.
      public: float MoveScale() const;

      /// \brief Set the scaling factor.
      /// \param[in] _scale The move scaling factor.
      public: void SetMoveScale(const float _scale);

      /// \brief Get the flag for mouse drag motion
      /// \return True if dragging, usually indicating a mouse move with
      /// mouse button pressed.
      public: bool Dragging() const;

      /// \brief Set the flag for mouse drag motion
      /// \param[in] _dragging The dragging flag.
      public: void SetDragging(const bool _dragging);

      /// \brief Get the event type.
      /// \return The EventType.
      public: EventType Type() const;

      /// \brief Set the event type.
      /// \param[in] _type The EventType.
      public: void SetType(const EventType _type) const;

      /// \brief Get the button which caused this event.
      /// \return The button which caused this event.
      public: MouseEvent::MouseButton Button() const;

      /// \brief Set the button which caused the event.
      /// \param[in] _button The button which caused this event.
      public: void SetButton(const MouseEvent::MouseButton _button) const;

      /// \brief Get the state of the buttons when the event was generated.
      /// \return The state of the buttons, which can be a bitwise
      /// combination of MouseEvent::MouseButton.
      public: unsigned int Buttons() const;

      /// \brief Set the state of the buttons when the event was generated.
      /// \param[in] _buttons The state of the buttons, which can be a bitwise
      /// combination of MouseEvent::MouseButton.
      public: void SetButtons(const unsigned int &_buttons);

      /// \brief Get the shift key press flag.
      /// \return True if the shift key is pressed.
      public: bool Shift() const;

      /// \brief Set the shift key press flag.
      /// \param[in] _shift The shift key press flag.
      public: void SetShift(const bool _shift) const;

      /// \brief Get the alt key press flag.
      /// \return True if the alt key is pressed.
      public: bool Alt() const;

      /// \brief Set the alt key press flag.
      /// \param[in] _alt The alt key flag.
      public: void SetAlt(const bool _alt);

      /// \brief Get the control key press flag.
      /// \return True if the control key is pressed.
      public: bool Control() const;

      /// \brief Set the control key press flag.
      /// \param[in] _control The control key flag.
      public: void SetControl(const bool _control) const;

      /// \brief Assignment operator
      /// \param[in] _other Other mouse event
      /// \return this
      public: MouseEvent &operator=(const MouseEvent &_other);

      /// \internal
      /// \brief Private data pointer
      private: MouseEventPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
