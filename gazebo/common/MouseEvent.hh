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
#ifndef _MOUSEEVENT_HH_
#define _MOUSEEVENT_HH_

#include "gazebo/math/Vector2i.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \class MouseEvent MouseEvent.hh common/common.hh
    /// \brief Generic description of a mouse event.
    class GAZEBO_VISIBLE MouseEvent
    {
      /// \brief Standard mouse buttons enumeration.
      public: enum Buttons {NO_BUTTON = 0x0, LEFT = 0x1, MIDDLE = 0x2,
                            RIGHT = 0x4};

      /// \brief Mouse event types enumeration.
      public: enum EventType {NO_EVENT, MOVE, PRESS, RELEASE, SCROLL};


      /// \brief Constructor.
      public: MouseEvent()
              : pos(0, 0), prevPos(0, 0), pressPos(0, 0), scroll(0, 0),
                moveScale(0.01), dragging(false), type(NO_EVENT), button(0),
                buttons(NO_BUTTON), shift(false), alt(false), control(false)
              {}


      /// \brief Mouse pointer position on the screen.
      public: math::Vector2i pos;

      /// \brief Previous position.
      public: math::Vector2i prevPos;

      /// \brief Position of button press.
      public: math::Vector2i pressPos;

      /// \brief Scroll position.
      public: math::Vector2i scroll;

      /// \brief Scaling factor.
      public: float moveScale;

      /// \brief Flag for mouse drag motion
      public: bool dragging;

      /// \brief Event type.
      public: EventType type;

      /// \brief The button which caused the event.
      public: unsigned int button;

      /// \brief State of the buttons when the event was generated.
      public: unsigned int buttons;

      /// \brief Shift key press flag.
      public: bool shift;

      /// \brief Alt key press flag.
      public: bool alt;

      /// \brief Control key press flag.
      public: bool control;
    };
    /// \}
  }
}
#endif
