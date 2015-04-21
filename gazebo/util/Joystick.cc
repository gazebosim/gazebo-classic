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

#include "gazebo/common/Console.hh"
#include "gazebo/util/JoystickPrivate.hh"
#include "gazebo/util/Joystick.hh"

using namespace gazebo;
using namespace util;

/////////////////////////////////////////////////
Joystick::Joystick()
  : dataPtr(new JoystickPrivate)
{
  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
}

/////////////////////////////////////////////////
Joystick::~Joystick()
{
  SDL_JoystickClose(this->dataPtr->joy);

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
bool Joystick::Init(const int _joyIndex)
{
  // FIXME: We don't need video, but without it SDL will fail to work in
  // SDL_WaitEvent()
  if (SDL_Init(SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK |
      SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) < 0)
  {
    std::cerr << "Unable to initialize SDL.\n";
    return false;
  }

  this->dataPtr->joy = SDL_JoystickOpen(_joyIndex);

  if (!this->dataPtr->joy)
  {
    std::cerr << "Unable to open SDL joystick device with index["
          << _joyIndex << "].\n";
    return false;
  }

  /*this->dataPtr->axesCount    = SDL_JoystickNumAxes(this->dataPtr->joy);
  this->dataPtr->buttonCount = SDL_JoystickNumButtons(this->dataPtr->joy);
  this->dataPtr->hatCount    = SDL_JoystickNumHats(this->dataPtr->joy);
  this->dataPtr->ballCount   = SDL_JoystickNumBalls(this->dataPtr->joy);
  */

  this->dataPtr->initialized = true;
  return true;
}

/////////////////////////////////////////////////
bool Joystick::Poll(msgs::Joysticks &_msg) const
{
  if (!this->dataPtr->initialized)
  {
    std::cerr << "Joystick is not initialized\n";
    return false;
  }

  bool result = false;
  SDL_Event event;

  while(SDL_PollEvent(&event))
  {
    msgs::Joystick *msg = _msg.add_joy();
    switch(event.type)
    {
      case SDL_JOYAXISMOTION:
        {
          result = true;
          int index = static_cast<int>(event.jaxis.axis);
          int value = static_cast<int>(event.jaxis.value);

          msgs::Joystick::Axis *axis = msg->add_analog_axis();
          axis->set_index(index);
          axis->set_value(value);

          // Debug
          // std::cout << "Joy axis motion\n"
          //   << "\tType[" << event.jaxis.type << "]\n"
          //   << "\tWhich[" << event.jaxis.which << "]\n"
          //   << "\tAxis[" << static_cast<int>(event.jaxis.axis) << "]\n"
          //   << "\tValue[" << static_cast<int>(event.jaxis.value) << "]\n";
          break;
        }

      case SDL_JOYHATMOTION:
        {
          result = true;
          // So far, only event.jhat.value seems useful
          int value = static_cast<int>(event.jhat.value);

          msgs::Joystick::Axis *axis = msg->add_digital_axis();
          axis->set_value(value);

          // Debug
          // std::cout << "Joy hat motion\n"
          //   << "\tType[" << event.jhat.type << "]\n"
          //   << "\tWhich[" << static_cast<int>(event.jhat.which) << "]\n"
          //   << "\tHat[" << static_cast<int>(event.jhat.hat) << "]\n"
          //   << "\tValue[" << static_cast<int>(event.jhat.value) << "]\n";
          break;
        }

      case SDL_JOYBUTTONUP:
      case SDL_JOYBUTTONDOWN:
        {
          result = true;
          int index = static_cast<int>(event.jbutton.button);
          int state = static_cast<int>(event.jbutton.state);

          msgs::Joystick::Button *button = msg->add_button();
          button->set_index(index);
          button->set_state(state);

          // Debug
          // std::cout << "Joy button down\n"
          //   << "\tType[" << event.jbutton.type << "]\n"
          //   << "\tWhich[" << static_cast<int>(event.jbutton.which)
          //   << "]\n"
          //   << "\tButton[" << static_cast<int>event.jbutton.button)
          //   << "]\n"
          //   << "\tState[" << static_cast<int>(event.jbutton.state)
          //   << "]\n";
          break;
        }

      default:
        break;
    }
  }

  return result;
}
