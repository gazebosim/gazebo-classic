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

#include "gz_joy.hh"

using namespace gazebo;

/////////////////////////////////////////////////
JoyCommand::JoyCommand()
  : Command("joy", "Joystick")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("run,r", "Run");

  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

  // FIXME: We don't need video, but without it SDL will fail to work in
  // SDL_WaitEvent()
  if (SDL_Init(SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK |
      SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) < 0)
  {
    gzerr << "Unable to initialize SDL.\n";
  }

  // \todo Make the argument to SDL_JoystickOpen a parameter. This should
  // allow independant control of multiple joysticks.
  this->joy = SDL_JoystickOpen(0);
  if (!this->joy)
  {
    gzerr << "Unable to open SDL joystick device.\n";
  }

  this->numAxes = SDL_JoystickNumAxes(joy);
  this->numButtons = SDL_JoystickNumButtons(joy);
  this->numHats    = SDL_JoystickNumHats(joy);
  this->numBalls   = SDL_JoystickNumBalls(joy);

  /*
  this->axes = (int*) calloc(num_axes,    sizeof(int));
  this->buttons = (unsigned short*) calloc(num_buttons, sizeof(unsigned short));
  this->hats = (unsigned short*) calloc(num_hats,    sizeof(unsigned short));
  this->balls = (unsigned short*) calloc(num_balls,   2*sizeof(int));
  */
}

/////////////////////////////////////////////////
void JoyCommand::HelpDetailed()
{
  std::cerr <<
    "\tPrint topic information to standard out. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool JoyCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  this->node.reset(new transport::Node());
  this->node->Init(worldName);

  while (this->running)
  {
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_JOYAXISMOTION:
          std::cout << "Joy axis motion\n";
          std::cout << "Axis[" << event.jaxis.axis << "] Value[" << event.jaxis.value << "] " << std::endl;
          break;

        case SDL_JOYHATMOTION:
          std::cout << "Joy hat motion\n";
          break;

        case SDL_JOYBALLMOTION:
          std::cout << "Joy ball motion\n";
          break;

        case SDL_JOYBUTTONDOWN:
          std::cout << "joy button down\n";
          break;

        case SDL_JOYBUTTONUP:
          std::cout << "joy button up\n";
          break;

        case SDL_QUIT:
          break;
      }
    }
  }


  return true;
}

/////////////////////////////////////////////////
bool JoyCommand::TransportRequired()
{
  return false;
}
