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

#ifndef _GAZEBO_JOYSTICK_HH_
#define _GAZEBO_JOYSTICK_HH_

#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  namespace util
  {
    // Forward declare private data class.
    class JoystickPrivate;

    /// \brief Generic interface to joysticks and gamepads using SDL.
    class GAZEBO_VISIBLE Joystick
    {
      /// \brief Constructor
      public: Joystick();

      /// \brief Destructor
      public: virtual ~Joystick();

      /// \brief Initialize and open a joystick.
      /// \param[in] _joyIndex Index of the joystick to open. This
      /// allows opening multiple joysticks.
      /// \return True if the joystick was successfully opened.
      public: bool Init(const int _joyIndex = 0);

      /// \brief Poll the joystick for new events.
      /// \param[in] _msgs Get all of the joystick events that were in the
      /// queue.
      /// \return True if there was at least one joystick event.
      public: bool Poll(msgs::Joysticks &_msg) const;

      /// \brief Private data pointer
      private: JoystickPrivate *dataPtr;
    };
  }
}
#endif
