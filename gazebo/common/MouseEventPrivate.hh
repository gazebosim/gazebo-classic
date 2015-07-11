/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_MOUSEEVENT_PRIVATE_HH_
#define _GAZEBO_MOUSEEVENT_PRIVATE_HH_

#include <ignition/math/Vector2.hh>
#include "gazebo/common/MouseEvent.hh"

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief Mouse event private data
    class MouseEventPrivate
    {
      /// \brief Constructor.
      public: MouseEventPrivate()
              : pos(0, 0), prevPos(0, 0), pressPos(0, 0), scroll(0, 0),
                moveScale(0.01), dragging(false), type(MouseEvent::NO_EVENT),
                button(MouseEvent::NO_BUTTON), buttons(MouseEvent::NO_BUTTON),
                shift(false), alt(false), control(false)
              {}

      /// \brief Mouse pointer position on the screen.
      public: ignition::math::Vector2i pos;

      /// \brief Previous position.
      public: ignition::math::Vector2i prevPos;

      /// \brief Position of button press.
      public: ignition::math::Vector2i pressPos;

      /// \brief Scroll position.
      public: ignition::math::Vector2i scroll;

      /// \brief Scaling factor.
      public: float moveScale;

      /// \brief Flag for mouse drag motion
      public: bool dragging;

      /// \brief Event type.
      public: MouseEvent::EventType type;

      /// \brief The button which caused the event.
      public: MouseEvent::MouseButton button;

      /// \brief State of the buttons when the event was generated.
      public: unsigned int buttons;

      /// \brief Shift key press flag.
      public: bool shift;

      /// \brief Alt key press flag.
      public: bool alt;

      /// \brief Control key press flag.
      public: bool control;
    };
  }
}
#endif
